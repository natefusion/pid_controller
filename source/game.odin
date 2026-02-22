/*
This file is the starting point of your game.

Some important procedures are:
- game_init_window: Opens the window
- game_init: Sets up the game state
- game_update: Run once per frame
- game_should_close: For stopping your game when close button is pressed
- game_shutdown: Shuts down game and frees memory
- game_shutdown_window: Closes window

The procs above are used regardless if you compile using the `build_release`
script or the `build_hot_reload` script. However, in the hot reload case, the
contents of this file is compiled as part of `build/hot_reload/game.dll` (or
.dylib/.so on mac/linux). In the hot reload cases some other procedures are
also used in order to facilitate the hot reload functionality:

- game_memory: Run just before a hot reload. That way game_hot_reload.exe has a
	pointer to the game's memory that it can hand to the new game DLL.
- game_hot_reloaded: Run after a hot reload so that the `g` global
	variable can be set to whatever pointer it was in the old DLL.

NOTE: When compiled as part of `build_release`, `build_debug` or `build_web`
then this whole package is just treated as a normal Odin package. No DLL is
created.
*/

package game

import "core:c"
import "core:fmt"
import "core:math"
import "core:strings"
import "core:unicode/utf8"
import "core:thread"
import "core:math/linalg"
// import "core:math/cmplx"
import rl "vendor:raylib"
import mu "vendor:microui"

PIXEL_WINDOW_HEIGHT :: 180
WIDTH :: 1280
HEIGHT :: 720

// DEETS
MAX_MOTOR_RPS : f64 : 70_300/*RPM*/ / 60.0
gravity :: 9.81 // m/s^2
ω :: MAX_MOTOR_RPS*math.TAU

hover_throttle :: 0.7 // estimated percent of max for hover
C_T :: 0.5//MASS * gravity / (ρ * (hover_throttle*hover_throttle*ω*ω) * D4)
ρ :: 1.225 // kg/m^3 (density of air)
D :: 0.031 // m (diamter of the propeller)
D4 :: D*D*D*D
D5 :: D4*D
PROP_PITCH :: 0.04826 // meter

// kg
MOTOR_MASS :: 0.00295
PROP_MASS :: 0.00019
BATT_MASS :: 0.05
ESP_MASS :: 0.04
CHASSIS_MASS :: 0.1
MASS :: (4*(MOTOR_MASS + PROP_MASS) + BATT_MASS + ESP_MASS + CHASSIS_MASS)

PIXELS_PER_M :: 30
MOTOR_TO_CENTER_M :: 0.07 // estimate (real)

DEFAULT_POS :: rl.Camera {
    position = {0.0, -4.0, 25.0},
    target = {0.0, 0.0, 0.0},
    up = {0.0, 0.0, 1.0},
    fovy = 90.0,
    projection = .PERSPECTIVE,
}

mouse_buttons_map := [mu.Mouse]rl.MouseButton{
	.LEFT    = .LEFT,
	.RIGHT   = .RIGHT,
	.MIDDLE  = .MIDDLE,
}

key_map := [mu.Key][2]rl.KeyboardKey{
	.SHIFT     = {.LEFT_SHIFT,   .RIGHT_SHIFT},
	.CTRL      = {.LEFT_CONTROL, .RIGHT_CONTROL},
	.ALT       = {.LEFT_ALT,     .RIGHT_ALT},
	.BACKSPACE = {.BACKSPACE,    .KEY_NULL},
	.DELETE    = {.DELETE,       .KEY_NULL},
	.RETURN    = {.ENTER,        .KP_ENTER},
	.LEFT      = {.LEFT,         .KEY_NULL},
	.RIGHT     = {.RIGHT,        .KEY_NULL},
	.HOME      = {.HOME,         .KEY_NULL},
	.END       = {.END,          .KEY_NULL},
	.A         = {.A,            .KEY_NULL},
	.X         = {.X,            .KEY_NULL},
	.C         = {.C,            .KEY_NULL},
	.V         = {.V,            .KEY_NULL},
}

MUI_State :: struct{
	mu_ctx: mu.Context,
	log_buf:         [1<<16]byte,
	log_buf_len:     int,
	log_buf_updated: bool,
	bg: mu.Color,
	atlas_texture: rl.RenderTexture2D,

	screen_width: c.int,
	screen_height: c.int,

	screen_texture: rl.RenderTexture2D,
}

Edit_Mode :: enum {
    None,
    Proportional,
    Integral,
    Derivative,
}

Pid_Type :: enum {
    Pitch,
    Roll,
    Yaw,
    None,
}

View_Mode :: enum {
    Top, Left, Follow,
}

Particle :: struct {
    motor_speed: f64,
    tangential_force : [3]f64,
    prop_force : [3]f64,
    prop_spin_dir : f64,
    position_old : [3]f64,
    position : [3]f64,
    mass: f64,
}

Link :: struct {
    p: int,
    p1: int,
    length: f64,
}

Super_Particle :: [6]Link

Recording_Size :: 1000

PID_Controller :: struct {
    Kp: f64,
    Ki: f64,
    Kd: f64,

    // Ti: f64,
    // Td: f64,
    // Ku: f64,
    // Tu: f64,

    data: [Recording_Size]f64,
    // io: [Recording_Size]complex64,
    data_idx: int,

    setpoint: f64,
    A: [3]f64,
    error: [3]f64,
    output: f64,
}

Game_3D :: struct {
    drone: Super_Particle,
    particles : [4]Particle,
    camera: rl.Camera3D,
    gyro : [3]f64,
    pid : [3]PID_Controller,
    throttle : f64,
    edit: Edit_Mode,
    selected_pid: Pid_Type,
    paused: bool,
    view_mode: View_Mode,
    slomo: bool,
    loop: bool,
    dt: f64,
    trajectory: [Recording_Size]f64,
    trajectory_idx: int,
}

Game_Memory :: struct {
	run: bool,
    state : MUI_State,
    g3d : Game_3D,
    running_thread: bool,
}

g: ^Game_Memory

set_game_3d_default :: proc(g: ^Game_3D, hard_reset: bool = false) {
    if hard_reset {
        g.camera = DEFAULT_POS
    }

    g.dt = 0.001

    g.particles = {
        { // front right
            prop_spin_dir = 1,
            position = {1,1,21},
        },
        { // front left
            prop_spin_dir = -1,
            position = {-1,1,21},
        },
        { // back right
            prop_spin_dir = -1,
            position = {1,-1,20},
        },
        { // back left
            prop_spin_dir = 1,
            position = {-1,-1,20},
        },
    }

    for &p in g.particles {
        p.position_old = p.position
        p.mass = MASS / 4
    }

    hypot :: 2*MOTOR_TO_CENTER_M * PIXELS_PER_M
    side :: hypot / math.SQRT_TWO

    for &d in g.pid {
        d.data = 0
        d.data_idx = 0
    }
    
    g.drone = {
        {p = 0, p1 = 1, length = side },
        {p = 0, p1 = 2, length = side },
        {p = 1, p1 = 3, length = side },
        {p = 3, p1 = 2, length = side },
        {p = 0, p1 = 3, length = hypot },
        {p = 1, p1 = 2, length = hypot },
    }
    g.paused = false
}

// fft :: proc(x: []complex64) {
//     N := len(x)
//     if N == 1 do return

//     x0 := make([]complex64, N/2)
//     x1 := make([]complex64, N/2)
//     defer delete(x0)
//     defer delete(x1)
    
//     for i := 0; 2 * i < N - 1; i+=1 {
//         x0[i] = x[2*i]
//         x1[i] = x[2*i+1]
//     }

//     fft(x0)
//     fft(x1)

//     ang := math.TAU / f64(N)
//     w := complex64(1)
//     wn := complex64(complex(math.cos(ang), math.sin(ang)))

//     for i := 0; 2 * i < N - 1; i+=1 {
//         x[i] = x0[i] + w * x1[i]
//         x[i + N/2] = x0[i] - w * x1[i]
//         w *= wn
//     }
// }

update_link :: proc(ps: []Particle, link : Link) {
    p_pos := &ps[link.p].position
    p1_pos := &ps[link.p1].position

    diff := p_pos^ - p1_pos^
    dist := linalg.length(diff)
    diff_factor := (link.length - dist) / dist
    offset := diff * diff_factor * 0.5

    p_pos^ += offset
    p1_pos^ -= offset
}

draw_links :: proc(g: ^Game_3D) {
    for l in g.drone {
        rl.DrawLine3D(cast_f32(g.particles[l.p].position), cast_f32(g.particles[l.p1].position), rl.MAGENTA) 
    }
}

init_pids :: proc(g: ^Game_3D) {
    init_pid(&g.pid[0], 0.4, 0.3, 0.1, g.dt)
    init_pid(&g.pid[1], 0.4, 0.3, 0.1, g.dt)
    init_pid(&g.pid[2], 0.7, 0.09, 0.0003, g.dt)
}

init_pid :: proc(using pid: ^PID_Controller, _Kp: f64 = 0.0, _Ki: f64 = 0.0, _Kd: f64 = 0.0, dt: f64 = 0.01) {
    Kp = _Kp
    Ki = _Ki
    Kd = _Kd
    setpoint = 0.0
    A = {
        Kp + Ki*dt + Kd/dt,
        -Kp - 2*Kd/dt,
        Kd/dt,
    }
    error = 0
    output = 0
}

reset_pid :: proc(using pid: ^PID_Controller) {
    error = 0
    output = 0
}

// autotune_pid :: proc(using pid: ^PID_Controller, dt: f64) {
//     /* 1. The ratio of output level to input level at low frequencies determines the gain parameter K of the model.
//        2. Observe the frequency Fu at which the phase passes through -pi radians (-180 degrees). The inverse of this frequency is the period of the oscillation, Tu.
//        3. Observe the plant gain Kc that occurs at the critical oscillation frequency Fu. The inverse of this is the gain margin Ku.
//      */

//     fft(io[:])
//     Ku = 1
//     Tu = 1
//     for i := 0; i < len(io)-1; i += 1 {
//         r1, θ1 := cmplx.polar(io[i])
//         r2, θ2 := cmplx.polar(io[i+1])

//         a := θ1 <= -math.PI && θ2 >= -math.PI
//         b := θ1 >= -math.PI && θ2 <= -math.PI
//         if a || b {
//             fs := 1/dt

//             Fu := f64(i) * fs / len(io)
//             Kc := f64(r1)

//             Ku = Kc == 0 ? 1 : 1.0/Kc;
//             Tu = Fu == 0 ? 1 : 1.0/Fu;

//             fmt.println("Found Ku and Tu!")
//             break
//         }
//         fmt.println(θ1)
//     }
    
//     Kp = (1.0/3.0) * Ku;
//     Ti = 0.5 * Tu;
//     Td = (1.0/3.0) * Tu;
    
//     Ki = Kp / Ti;
//     Kd = Kp * Td;

//     io = 0 // fft algo is in place, so reset to allow for more data
// }

update_pid :: proc(measured_value: f64, pid: ^PID_Controller, dt: f64) {
    pid.A[0] = pid.Kp + pid.Ki*dt + pid.Kd/dt
    pid.A[1] = -pid.Kp - 2*pid.Kd/dt
    pid.A[2] = pid.Kd / dt
    pid.error[2] = pid.error[1]
    pid.error[1] = pid.error[0]
    pid.error[0] = pid.setpoint - measured_value
    pid.error = linalg.clamp(pid.error, -1, 1)
    pid.output += linalg.dot(pid.A, pid.error)
}

draw_pid_stats :: proc(pid: ^PID_Controller, start_y: i32 = 10) -> (end_y: i32) {
    end_y = start_y
    
    str := fmt.caprintf("PID OUT: %f", pid.output)
    rl.DrawText(str, 10, end_y, 20, rl.RED)

    end_y += 20

    str = fmt.caprintf("PID ERR: %f", pid.error)
    rl.DrawText(str, 10, end_y, 20, rl.BLACK)

    end_y += 20

    str = fmt.caprintf("Kp: %f, Ki: %f, Kd: %f", pid.Kp, pid.Ki, pid.Kd)
    rl.DrawText(str, 10, end_y, 20, rl.BLACK)

    end_y += 20

    str = fmt.caprintf("setpoint: %f", pid.setpoint)
    rl.DrawText(str, 10, end_y, 20, rl.BLACK)

    end_y += 50

    return
}

update_particle :: proc(p: ^Particle, dt: f64) {
    gravity :: [3]f64{0, 0, -9.81}
    force := p.tangential_force + p.prop_force
    temp := p.position
    a := force / p.mass + gravity
    x := a * dt * dt
    p.position = 2*p.position - p.position_old + x
    p.position_old = temp
    
}

handle_ground_collision_3d :: proc(p: ^Particle) {
    if (p.position.z <= 0.1) {
        p.position.z = 0.1
        p.position_old.z = 0.1
    }
}

update_particles :: proc(particles: []Particle, dt: f64) {
    for &p in particles {
        update_particle(&p, dt)
        handle_ground_collision_3d(&p)
    }
}

reset :: proc(g: ^Game_3D, hard: bool = false) {
    if hard {
        init_pids(g)
    } else {
        for &p in g.pid do reset_pid(&p)
    }
    set_game_3d_default(g, hard)

}

handle_input_3d :: proc(game: ^Game_3D) {
    if rl.IsKeyDown(.P) {
        game.edit = .Proportional
    } else if rl.IsKeyDown(.I) {
        game.edit = .Integral
    } else if rl.IsKeyDown(.F) {
        game.edit = .Derivative
    } else if rl.IsKeyDown(.N) {
        game.edit = .None
    }

    mw : f64 = 0
    if rl.IsKeyPressed(.LEFT_BRACKET) {
        mw -= 0.1
    }
    if rl.IsKeyPressed(.RIGHT_BRACKET) {
        mw += 0.1
    }

    if rl.IsKeyPressed(.M) {
        game.loop = !game.loop
    }

    if rl.IsKeyPressed(.ONE) {
        game.selected_pid = .Pitch
    } else if rl.IsKeyPressed(.TWO) {
        game.selected_pid = .Roll
    } else if rl.IsKeyPressed(.THREE) {
        game.selected_pid = .Yaw
    } else if rl.IsKeyPressed(.ZERO) {
        game.selected_pid = .None
    }

    if game.selected_pid == .None {
        for &p in game.pid {
            switch game.edit {
            case .Proportional:
                p.Kp += mw
            case .Integral:
                p.Ki += mw
            case .Derivative:
                p.Kd += mw
            case .None:
            }
        }
    } else {
        p := &game.pid[game.selected_pid]
        switch game.edit {
        case .Proportional:
            p.Kp += mw
        case .Integral:
            p.Ki += mw
        case .Derivative:
            p.Kd += mw
        case .None:
        }
    }

    

    if rl.IsKeyPressed(.COMMA) {
        game.camera = DEFAULT_POS
    }

    if rl.IsKeyPressed(.V) {
        game.slomo = !game.slomo
    }

    if rl.IsKeyPressed(.EQUAL) {
        game.throttle += 0.1
        game.throttle = clamp(game.throttle, 0, 1)
    } else if rl.IsKeyPressed(.MINUS) {
        game.throttle -= 0.1
        game.throttle = clamp(game.throttle, 0, 1)
    }

    if rl.IsKeyPressed(.SEMICOLON) {
        game.pid[2].setpoint += 0.1 * math.PI
        game.pid[2].setpoint = clamp(game.pid[2].setpoint, -math.PI, math.PI)
    } else if rl.IsKeyPressed(.APOSTROPHE) {
        game.pid[2].setpoint -= 0.1 * math.PI
        game.pid[2].setpoint = clamp(game.pid[2].setpoint, -math.PI, math.PI)
    }

    if rl.IsKeyPressed(.PERIOD) || rl.IsKeyPressed(.SLASH) {
        if rl.IsKeyPressed(.PERIOD) {
            if rl.IsKeyDown(.LEFT_SHIFT) {
                game.view_mode = .Follow
            } else {
                if game.view_mode == .Top do game.view_mode = .Left
                else do game.view_mode = .Top
            }
        }

        c := cast_f32(drone_center(game))
        s := cast_f32(drone_side(game))
        // raylib quirk target and position x,y cannot match exactly!!
        p := game.view_mode == .Top ? c + {0.0001, 0.0001, 3} : s + {3, 0.001, 0.0001}
        t := game.view_mode == .Top ? c : s
        game.camera = {
            position = p,
            target = t,
            up = {0.0, 0.0, 1.0},
            fovy = 90.0,
            projection = .PERSPECTIVE,
        }

    }
    
    if rl.IsKeyPressed(.L) {
        game.paused = !game.paused
    }

    if rl.IsKeyPressed(.R) {
        hr := rl.IsKeyDown(.LEFT_SHIFT)
        reset(game, hr)
    }

    if rl.IsKeyPressed(.J) {
        if !g.running_thread do thread.run(show_future_events_thread)
    }
    
}

draw_force_vectors :: proc(g: ^Game_3D) {
    for p in g.particles {
        end_pos := p.position - p.prop_force*20
        rl.DrawLine3D(cast_f32(p.position), cast_f32(end_pos), rl.RED)

        end_pos = p.position - p.tangential_force
        rl.DrawLine3D(cast_f32(p.position), cast_f32(end_pos), rl.RED)
    }
}

update_gyro :: proc(g: ^Game_3D) {
    c := drone_center(g)
    s := drone_side(g)
    f := drone_front(g)

    fc := f - c
    sc := s - c
    g.gyro[0] = linalg.angle_between(fc.yz, [2]f64{0, -1}) - math.PI / 2
    g.gyro[1] = linalg.angle_between(sc.xz, [2]f64{0, -1}) - math.PI / 2
    
    g.gyro[2] = linalg.angle_between(sc.xy, [2]f64{1, 0}) * math.sign(linalg.cross(sc.xy, [2]f64{1, 0}))
    
    // expanded rotation matrix
    // rotate pitch and roll by 45 deg
    rot_amount := math.PI / 4 - (g.gyro[2])
    new_pitch := g.gyro[0] * math.cos(rot_amount) - g.gyro[1] * math.sin(rot_amount)
    new_roll :=  g.gyro[0] * math.sin(rot_amount) + g.gyro[1] * math.cos(rot_amount)
    g.gyro[0] = new_pitch
    g.gyro[1] = new_roll
}

// https://web.mit.edu/16.unified/www/FALL/thermodynamics/notes/node86.html
calc_thrust :: proc(val: f64) -> (F_thrust: f64) {
    assert(val >= 0.0)
    assert(val <= 1.0)
    
    n := val * MAX_MOTOR_RPS
    F_thrust = C_T * ρ * n*n * D4
    return
}


calc_tangential_force :: proc(val: f64) -> (F_tangential: f64) {
    assert(val >= 0.0)
    assert(val <= 1.0)

    η :: 0.6 // efficiency (0.6 - 0.8)
    C_Q :: (D / (math.TAU * PROP_PITCH)) * η * C_T
    K_T :: C_Q * ρ * D5 / (math.TAU * math.TAU) // kg*m^2 (aerodynamic drag coefficient)
    ω :: MAX_MOTOR_RPS*math.TAU
    MAX_TORQUE :: K_T * ω*ω
    T_m := val * MAX_TORQUE
    F_tangential = T_m / MOTOR_TO_CENTER_M
    return
}

handle_pid3d :: proc(g: ^Game_3D) {
    for i in 0..<3 do update_pid(g.gyro[i], &g.pid[i], g.dt)

    for &p, i in g.pid {
        p.data[p.data_idx] = g.gyro[i]
        // p.io[p.data_idx] = g.gyro[i] == 0 ? 0 : complex(p.output / g.gyro[i], 0)
        // if p.io[p.data_idx] != p.io[p.data_idx] do fmt.println(p.output, g.gyro[i])

        // if p.data_idx == len(p.data) - 1 {
        //     autotune_pid(&p)
        // }

        p.data_idx = (p.data_idx + 1) % len(p.data)
    }

    ps := &g.particles
    po := clamp(g.pid[0].output, -1, 1)
    ro := clamp(g.pid[1].output, -1, 1)
    yo := clamp(g.pid[2].output, -1, 1)

    ps[0].motor_speed = g.throttle + ro + ps[0].prop_spin_dir * yo
    ps[1].motor_speed = g.throttle + po + ps[1].prop_spin_dir * yo
    ps[2].motor_speed = g.throttle - po + ps[2].prop_spin_dir * yo
    ps[3].motor_speed = g.throttle - ro + ps[3].prop_spin_dir * yo

    c := drone_center(g)
    for &p in ps {
        p.motor_speed = clamp(p.motor_speed, 0, 1)
        p.prop_force = drone_normal(g) * calc_thrust(p.motor_speed)

        plane := linalg.normalize(linalg.cross(p.position - c, drone_normal(g)))
        p.tangential_force = p.prop_spin_dir * plane * calc_tangential_force(p.motor_speed)
    }
}


draw_drone :: proc(g: ^Game_3D) {
    radius :: 0.5
    colors := [4]rl.Color{rl.RED, rl.GREEN, rl.PURPLE, rl.BLUE}
    for p, i in g.particles {
        if !(i == 0 || i == 3) do continue 
        if p.prop_spin_dir > 0 {
            rl.DrawSphere({cast(f32)p.position.x, cast(f32)p.position.y, cast(f32)p.position.z},
                          cast(f32)radius,  colors[i])
        } else {
            rl.DrawCube({cast(f32)p.position.x, cast(f32)p.position.y, cast(f32)p.position.z},
                        cast(f32)radius*2, cast(f32)radius*2, cast(f32)radius*2, colors[i])
        }
    }
}

draw_gyro :: proc(gyro: [3]f64) {
    gyro_deg := gyro * 180 / math.PI
    rl.DrawText(fmt.caprintf("Pitch: %f\nRoll:   %f\nYaw:   %f", gyro_deg.x, gyro_deg.y, gyro_deg.z), 10, 10, 20, rl.BLACK)
}

cast_f32 :: proc(a: [3]f64) -> [3]f32 {
    return {cast(f32)a[0], cast(f32)a[1], cast(f32)a[2]}
}

drone_center :: proc(g: ^Game_3D) -> [3]f64 {
    diff1 := g.particles[1].position - g.particles[0].position
    diff2 := g.particles[2].position - g.particles[0].position

    return g.particles[0].position + diff1/2 + diff2/2
}

drone_front :: proc(g: ^Game_3D) -> [3]f64 {
    diff1 := g.particles[1].position - g.particles[0].position
    return g.particles[0].position + diff1/2
}

drone_side :: proc(g: ^Game_3D) -> [3]f64 {
    diff1 := g.particles[2].position - g.particles[0].position
    return g.particles[0].position + diff1/2
}

drone_normal :: proc(g: ^Game_3D) -> [3]f64 {
    // the positions of the particle should be determined by the normal, not the other way around
    c := drone_center(g)
    a := g.particles[0].position - c
    b := g.particles[1].position - c
    return linalg.normalize(linalg.cross(a, b))
}

draw_normal :: proc(g: ^Game_3D) {
    c := cast_f32(drone_center(g))
    normal := cast_f32(drone_normal(g))
    f := cast_f32(drone_front(g))
    s := cast_f32(drone_side(g))
    rl.DrawSphere(c + normal, 0.125, rl.RED)
    rl.DrawSphere(c, 0.125, rl.PURPLE)
    rl.DrawSphere(f, 0.125, rl.YELLOW)
    rl.DrawSphere(s, 0.125, rl.WHITE)
    rl.DrawLine3D(c, c + [3]f32{0, 0, -1}, rl.BLUE)
    rl.DrawLine3D(c, f, rl.BLUE)
    rl.DrawLine3D(c, s, rl.BLUE)
    rl.DrawLine3D(c, c + [3]f32{1, 0, 0}, rl.BLUE)
}

DrawPlotSimple :: proc(bounds: rl.Rectangle, name: cstring, data: []f64, setpoint: f64, pos: int) {
    h_shift :: 2.0
    rl.DrawRectangleRoundedLinesEx(bounds, 0.0, 0, 1, rl.BLACK)
    rl.DrawText(name, i32(bounds.x) + 2, i32(bounds.y) + 2, 10, rl.GRAY)
    
    h_offset := bounds.y + bounds.height
    h_ratio := bounds.height / 4.0

    posval := f32(pos) * bounds.width / f32(len(data))
    
    v := [2]f32{bounds.x, clamp(h_offset - h_ratio*(f32(data[0]) + h_shift), bounds.y, h_offset) }
    for i in 0..<bounds.width {
        idx := int(f32(i) * f32(len(data)) / bounds.width)
        y := f32(data[idx]) + h_shift
        pv := v
        v = [2]f32{ bounds.x + i, clamp(h_offset - h_ratio*y, bounds.y, h_offset) }
        rl.DrawLineEx(pv, v, 2, i < posval ? rl.RED : rl.Color {255,0,0,32})
    }
    
    rl.DrawLineEx({bounds.x + posval, bounds.y}, {bounds.x + posval, bounds.y + bounds.height}, 1, rl.GREEN)

    yval := clamp(h_offset - h_ratio*(f32(setpoint) + h_shift), bounds.y, h_offset)
    rl.DrawLineEx({bounds.x, yval }, {bounds.x + bounds.width, yval}, 1, rl.BLUE)
}


draw_3d :: proc(g: ^Game_3D) {
    if (g.view_mode == .Follow) {
        g.camera = {
            position = cast_f32(drone_center(g)) + 10,
            target = cast_f32(drone_center(g)),
            up = {0.0, 0.0, 1.0},
            fovy = 90.0,
            projection = .PERSPECTIVE,
        }
    }
    rl.UpdateCamera(&g.camera, .FREE)

    rl.BeginMode3D(g.camera)
    draw_drone(g)
    rl.DrawGrid(10, 2.0)
    draw_normal(g)
    draw_links(g)
    draw_force_vectors(g)
    rl.EndMode3D()

    draw_gyro(g.gyro)
    end_y : i32 = 100
    end_y = draw_pid_stats(&g.pid[0], end_y)
    end_y = draw_pid_stats(&g.pid[1], end_y)
    end_y = draw_pid_stats(&g.pid[2], end_y)
    rl.DrawText(fmt.caprintf("edit  mode: %v", g.edit), 10, end_y, 20, rl.GREEN)
    rl.DrawText(fmt.caprintf("selected pid: %v", g.selected_pid), 10, end_y+20, 20, rl.GREEN)
    rl.DrawText(fmt.caprintf("Pos %.3f\n", drone_center(g)), 10, end_y+40, 20, rl.GREEN)
    for p, i in g.particles {
        rl.DrawText(fmt.caprintf("Motor speed %v: %.3f\n", i, p.motor_speed), 10, end_y+i32(20*(3+i)), 20, rl.GREEN)
    }

    rl.DrawText(fmt.caprintf("Rot: %f", g.pid[2].setpoint * 180 / math.PI), 10, HEIGHT - 60, 20, rl.GREEN)
    rl.DrawText(fmt.caprintf("Throttle: %f", g.throttle), 10, HEIGHT - 40, 20, rl.GREEN)

    if g.loop do rl.DrawText(fmt.caprintf("[Loop (tm)]"), 230, HEIGHT - 20, 20, rl.GREEN)
    if g.slomo do rl.DrawText(fmt.caprintf("[Slomo (tm)]"), 110, HEIGHT - 20, 20, rl.GREEN)
    if g.paused do rl.DrawText(fmt.caprintf("[Paused]"), 10, HEIGHT - 20, 20, rl.GREEN)

    for &p, i in g.pid {
        rr : f32 : 200
        rec := rl.Rectangle{
            x = WIDTH - rr - 2,
            y = 2 + f32(i) * (rr + 4),
            width = rr,
            height = rr,
        }

        DrawPlotSimple(rec, fmt.caprint(Pid_Type(i)), p.data[:], g.pid[i].setpoint, p.data_idx)
    }
}

update_3d :: proc(g: ^Game_3D) {
    update_gyro(g)
    handle_pid3d(g)
    update_particles(g.particles[:], g.dt)
    for link in g.drone do update_link(g.particles[:], link)
}

micro_ui_stuff :: proc() {
    ctx := &g.state.mu_ctx
    mouse_pos := rl.GetMousePosition()
	mouse_x, mouse_y := i32(mouse_pos.x), i32(mouse_pos.y)
	mu.input_mouse_move(ctx, mouse_x, mouse_y)

	mouse_wheel_pos := rl.GetMouseWheelMoveV()
	mu.input_scroll(ctx, i32(mouse_wheel_pos.x) * 30, i32(mouse_wheel_pos.y) * -30)

	for button_rl, button_mu in mouse_buttons_map {
		switch {
		case rl.IsMouseButtonPressed(button_rl):
			mu.input_mouse_down(ctx, mouse_x, mouse_y, button_mu)
		case rl.IsMouseButtonReleased(button_rl):
			mu.input_mouse_up  (ctx, mouse_x, mouse_y, button_mu)
		}
	}

	for keys_rl, key_mu in key_map {
		for key_rl in keys_rl {
			switch {
			case key_rl == .KEY_NULL:
				// ignore
			case rl.IsKeyPressed(key_rl), rl.IsKeyPressedRepeat(key_rl):
				mu.input_key_down(ctx, key_mu)
			case rl.IsKeyReleased(key_rl):
				mu.input_key_up  (ctx, key_mu)
			}
		}
	}

	{
		buf: [512]byte
		n: int
		for n < len(buf) {
			c := rl.GetCharPressed()
			if c == 0 {
				break
			}
			b, w := utf8.encode_rune(c)
			n += copy(buf[n:], b[:w])
		}
		mu.input_text(ctx, string(buf[:n]))
	}

	mu.begin(ctx)
	all_windows(ctx)
	mu.end(ctx)
}

game_3d :: proc(game: ^Game_3D) {
    if !game.paused {
        update_3d(game)
    }

    micro_ui_stuff()

    // more micro ui stuff
    render(game, &g.state.mu_ctx)    // draw_3d(game)
    
    handle_input_3d(game)
    
    if game.loop {
        if game.pid[0].data_idx == len(game.pid[0].data) - 1 {
            reset(game)
        }
    }
    if game.slomo {
        game.dt = 0.001
    } else {
        game.dt = f64(rl.GetFrameTime())
    }

}

render :: proc (game: ^Game_3D, ctx: ^mu.Context) {
	render_texture :: proc "contextless" (renderer: rl.RenderTexture2D, dst: ^rl.Rectangle, src: mu.Rect, color: rl.Color) {
		dst.width = f32(src.w)
		dst.height = f32(src.h)

		rl.DrawTextureRec(
			texture  = g.state.atlas_texture.texture,
			source   = {f32(src.x), f32(src.y), f32(src.w), f32(src.h)},
			position = {dst.x, dst.y},
			tint     = color,
		)
	}

	to_rl_color :: proc "contextless" (in_color: mu.Color) -> (out_color: rl.Color) {
		return {in_color.r, in_color.g, in_color.b, in_color.a}
	}

	height := rl.GetScreenHeight()

	rl.BeginTextureMode(g.state.screen_texture)
	rl.EndScissorMode()
	rl.ClearBackground(to_rl_color(g.state.bg))

	command_backing: ^mu.Command
	for variant in mu.next_command_iterator(ctx, &command_backing) {
		switch cmd in variant {
		case ^mu.Command_Text:
			dst := rl.Rectangle{f32(cmd.pos.x), f32(cmd.pos.y), 0, 0}
			for ch in cmd.str {
				if ch&0xc0 != 0x80 {
					r := min(int(ch), 127)
					src := mu.default_atlas[mu.DEFAULT_ATLAS_FONT + r]
					render_texture(g.state.screen_texture, &dst, src, to_rl_color(cmd.color))
					dst.x += dst.width
				}
			}
		case ^mu.Command_Rect:
			rl.DrawRectangle(cmd.rect.x, cmd.rect.y, cmd.rect.w, cmd.rect.h, to_rl_color(cmd.color))
		case ^mu.Command_Icon:
			src := mu.default_atlas[cmd.id]
			x := cmd.rect.x + (cmd.rect.w - src.w)/2
			y := cmd.rect.y + (cmd.rect.h - src.h)/2
			render_texture(g.state.screen_texture, &rl.Rectangle {f32(x), f32(y), 0, 0}, src, to_rl_color(cmd.color))
		case ^mu.Command_Clip:
			rl.BeginScissorMode(cmd.rect.x, height - (cmd.rect.y + cmd.rect.h), cmd.rect.w, cmd.rect.h)
		case ^mu.Command_Jump:
			unreachable()
		}
	}
	rl.EndTextureMode()

	rl.BeginDrawing()
	rl.ClearBackground(rl.RAYWHITE)
    draw_3d(game)
    
	rl.DrawTextureRec(
		texture  = g.state.screen_texture.texture,
		source   = {0, 0, f32(g.state.screen_width), -f32(g.state.screen_height)},
		position = {0, 0},
		tint     = rl.WHITE,
	)
	rl.EndDrawing()
}

show_future_events_thread :: proc() {
    g.running_thread = true
    defer g.running_thread = false
    
    fmt.println("Starting Thread!")
    defer fmt.println("Just finished the thread... ")
    
    new_game := new_clone(g.g3d)
    defer free(new_game)
    
    for _ in 0..<Recording_Size {
        update_3d(new_game)
        thread.yield()
    }

    is_paused := g.g3d.paused
    defer g.g3d.paused = is_paused
    
    if !is_paused do g.g3d.paused = true // does this even do what I think it does?

    g.g3d.trajectory = new_game.trajectory
    g.g3d.trajectory_idx = new_game.trajectory_idx
    for &p, i in g.g3d.pid {
        p.data = new_game.pid[i].data
        p.data_idx = new_game.pid[i].data_idx
    }
}

@(export)
game_update :: proc() {
    game_3d(&g.g3d)

	// Everything on tracking allocator is valid until end-of-frame.
	free_all(context.temp_allocator)
}

micro_ui_init :: proc() {
    ctx := &g.state.mu_ctx
	mu.init(ctx,
		set_clipboard = proc(user_data: rawptr, text: string) -> (ok: bool) {
			cstr := strings.clone_to_cstring(text)
			rl.SetClipboardText(cstr)
			delete(cstr)
			return true
		},
		get_clipboard = proc(user_data: rawptr) -> (text: string, ok: bool) {
			cstr := rl.GetClipboardText()
			if cstr != nil {
				text = string(cstr)
				ok = true
			}
			return
		},
	)

	ctx.text_width = mu.default_atlas_text_width
	ctx.text_height = mu.default_atlas_text_height

	g.state.atlas_texture = rl.LoadRenderTexture(c.int(mu.DEFAULT_ATLAS_WIDTH), c.int(mu.DEFAULT_ATLAS_HEIGHT))
	// defer rl.UnloadRenderTexture(g.state.atlas_texture)

	image := rl.GenImageColor(c.int(mu.DEFAULT_ATLAS_WIDTH), c.int(mu.DEFAULT_ATLAS_HEIGHT), rl.Color{0, 0, 0, 0})
	// defer rl.UnloadImage(image)

	for alpha, i in mu.default_atlas_alpha {
		x := i % mu.DEFAULT_ATLAS_WIDTH
		y := i / mu.DEFAULT_ATLAS_WIDTH
		color := rl.Color{255, 255, 255, alpha}
		rl.ImageDrawPixel(&image, c.int(x), c.int(y), color)
	}

	rl.BeginTextureMode(g.state.atlas_texture)
	rl.UpdateTexture(g.state.atlas_texture.texture, rl.LoadImageColors(image))
	rl.EndTextureMode()

	g.state.screen_texture = rl.LoadRenderTexture(g.state.screen_width, g.state.screen_height)
	// defer rl.UnloadRenderTexture(g.state.screen_texture)
}

@(export)
game_init_window :: proc() {
	rl.SetConfigFlags({.WINDOW_RESIZABLE, .VSYNC_HINT})
	rl.InitWindow(WIDTH, HEIGHT, "Pid Controller")
	rl.SetTargetFPS(60)
	rl.SetExitKey(nil)
    // rl.HideCursor()
    // rl.DisableCursor()
}
 

@(export)
game_init :: proc() {
	g = new(Game_Memory)

	g^ = Game_Memory {
		run = true,
        state = {
            screen_width = WIDTH,
	        screen_height = HEIGHT,
	        bg = {90, 95, 100, 0},
        },
        running_thread = false,
	}

    micro_ui_init()

    set_game_3d_default(&g.g3d)

    init_pids(&g.g3d)
	game_hot_reloaded(g)

}

@(export)
game_should_run :: proc() -> bool {
	when ODIN_OS != .JS {
		// Never run this proc in browser. It contains a 16 ms sleep on web!
		if rl.WindowShouldClose() {
			return false
		}
	}

	return g.run
}

@(export)
game_shutdown :: proc() {
	free(g)
}

@(export)
game_shutdown_window :: proc() {
	rl.CloseWindow()
}

@(export)
game_memory :: proc() -> rawptr {
	return g
}

@(export)
game_memory_size :: proc() -> int {
	return size_of(Game_Memory)
}

@(export)
game_hot_reloaded :: proc(mem: rawptr) {
	g = (^Game_Memory)(mem)

	// Here you can also set your own global variables. A good idea is to make
	// your global variables into pointers that point to something inside `g`.
}

@(export)
game_force_reload :: proc() -> bool {
	return rl.IsKeyPressed(.F5)
}

@(export)
game_force_restart :: proc() -> bool {
	return rl.IsKeyPressed(.F6)
}

// In a web build, this is called when browser changes size. Remove the
// `rl.SetWindowSize` call if you don't want a resizable game.
game_parent_window_size_changed :: proc(w, h: int) {
	rl.SetWindowSize(i32(w), i32(h))
}

write_log :: proc(str: string) {
	g.state.log_buf_len += copy(g.state.log_buf[g.state.log_buf_len:], str)
	g.state.log_buf_len += copy(g.state.log_buf[g.state.log_buf_len:], "\n")
	g.state.log_buf_updated = true
}

read_log :: proc() -> string {
	return string(g.state.log_buf[:g.state.log_buf_len])
}

u8_slider :: proc(ctx: ^mu.Context, val: ^u8, lo, hi: u8) -> (res: mu.Result_Set) {
	mu.push_id(ctx, uintptr(val))

	@static tmp: mu.Real
	tmp = mu.Real(val^)
	res = mu.slider(ctx, &tmp, mu.Real(lo), mu.Real(hi), 0, "%.0f", {.ALIGN_CENTER})
	val^ = u8(tmp)
	mu.pop_id(ctx)
	return
}

all_windows :: proc(ctx: ^mu.Context) {
	@static opts := mu.Options{.NO_CLOSE}

	if mu.window(ctx, "Demo Window", {40, 40, 300, 450}, opts) {
		if .ACTIVE in mu.header(ctx, "Window Info") {
			win := mu.get_current_container(ctx)
			mu.layout_row(ctx, {54, -1}, 0)
			mu.label(ctx, "Position:")
			mu.label(ctx, fmt.tprintf("%d, %d", win.rect.x, win.rect.y))
			mu.label(ctx, "Size:")
			mu.label(ctx, fmt.tprintf("%d, %d", win.rect.w, win.rect.h))
		}

		if .ACTIVE in mu.header(ctx, "Window Options") {
			mu.layout_row(ctx, {120, 120, 120}, 0)
			for opt in mu.Opt {
				state := opt in opts
				if .CHANGE in mu.checkbox(ctx, fmt.tprintf("%v", opt), &state)  {
					if state {
						opts += {opt}
					} else {
						opts -= {opt}
					}
				}
			}
		}

		if .ACTIVE in mu.header(ctx, "Test Buttons", {.EXPANDED}) {
			mu.layout_row(ctx, {86, -110, -1})
			mu.label(ctx, "Test buttons 1:")
			if .SUBMIT in mu.button(ctx, "Button 1") { write_log("Pressed button 1") }
			if .SUBMIT in mu.button(ctx, "Button 2") { write_log("Pressed button 2") }
			mu.label(ctx, "Test buttons 2:")
			if .SUBMIT in mu.button(ctx, "Button 3") { write_log("Pressed button 3") }
			if .SUBMIT in mu.button(ctx, "Button 4") { write_log("Pressed button 4") }
		}

		if .ACTIVE in mu.header(ctx, "Tree and Text", {.EXPANDED}) {
			mu.layout_row(ctx, {140, -1})
			mu.layout_begin_column(ctx)
			if .ACTIVE in mu.treenode(ctx, "Test 1") {
				if .ACTIVE in mu.treenode(ctx, "Test 1a") {
					mu.label(ctx, "Hello")
					mu.label(ctx, "world")
				}
				if .ACTIVE in mu.treenode(ctx, "Test 1b") {
					if .SUBMIT in mu.button(ctx, "Button 1") { write_log("Pressed button 1") }
					if .SUBMIT in mu.button(ctx, "Button 2") { write_log("Pressed button 2") }
				}
			}
			if .ACTIVE in mu.treenode(ctx, "Test 2") {
				mu.layout_row(ctx, {53, 53})
				if .SUBMIT in mu.button(ctx, "Button 3") { write_log("Pressed button 3") }
				if .SUBMIT in mu.button(ctx, "Button 4") { write_log("Pressed button 4") }
				if .SUBMIT in mu.button(ctx, "Button 5") { write_log("Pressed button 5") }
				if .SUBMIT in mu.button(ctx, "Button 6") { write_log("Pressed button 6") }
			}
			if .ACTIVE in mu.treenode(ctx, "Test 3") {
				@static checks := [3]bool{true, false, true}
				mu.checkbox(ctx, "Checkbox 1", &checks[0])
				mu.checkbox(ctx, "Checkbox 2", &checks[1])
				mu.checkbox(ctx, "Checkbox 3", &checks[2])

			}
			mu.layout_end_column(ctx)

			mu.layout_begin_column(ctx)
			mu.layout_row(ctx, {-1})
			mu.text(ctx,
				"Lorem ipsum dolor sit amet, consectetur adipiscing "+
				"elit. Maecenas lacinia, sem eu lacinia molestie, mi risus faucibus "+
				"ipsum, eu varius magna felis a nulla.",
			)
			mu.layout_end_column(ctx)
		}

		if .ACTIVE in mu.header(ctx, "Background Colour", {.EXPANDED}) {
			mu.layout_row(ctx, {-78, -1}, 68)
			mu.layout_begin_column(ctx)
			{
				mu.layout_row(ctx, {46, -1}, 0)
				mu.label(ctx, "Red:");   u8_slider(ctx, &g.state.bg.r, 0, 255)
				mu.label(ctx, "Green:"); u8_slider(ctx, &g.state.bg.g, 0, 255)
				mu.label(ctx, "Blue:");  u8_slider(ctx, &g.state.bg.b, 0, 255)
			}
			mu.layout_end_column(ctx)

			r := mu.layout_next(ctx)
			mu.draw_rect(ctx, r, g.state.bg)
			mu.draw_box(ctx, mu.expand_rect(r, 1), ctx.style.colors[.BORDER])
			mu.draw_control_text(ctx, fmt.tprintf("#%02x%02x%02x", g.state.bg.r, g.state.bg.g, g.state.bg.b), r, .TEXT, {.ALIGN_CENTER})
		}
	}



	if mu.window(ctx, "Log Window", {350, 40, 300, 200}, opts) {
		mu.layout_row(ctx, {-1}, -28)
		mu.begin_panel(ctx, "Log")
		mu.layout_row(ctx, {-1}, -1)
		mu.text(ctx, read_log())
		if g.state.log_buf_updated {
			panel := mu.get_current_container(ctx)
			panel.scroll.y = panel.content_size.y
			g.state.log_buf_updated = false
		}
		mu.end_panel(ctx)

		@static buf: [128]byte
		@static buf_len: int
		submitted := false
		mu.layout_row(ctx, {-70, -1})
		if .SUBMIT in mu.textbox(ctx, buf[:], &buf_len) {
			mu.set_focus(ctx, ctx.last_id)
			submitted = true
		}
		if .SUBMIT in mu.button(ctx, "Submit") {
			submitted = true
		}
		if submitted {
			write_log(string(buf[:buf_len]))
			buf_len = 0
		}
	}

	if mu.window(ctx, "Style Window", {350, 250, 300, 240}) {
		@static colors := [mu.Color_Type]string{
			.TEXT         = "text",
			.BORDER       = "border",
			.WINDOW_BG    = "window bg",
			.TITLE_BG     = "title bg",
			.TITLE_TEXT   = "title text",
			.PANEL_BG     = "panel bg",
			.BUTTON       = "button",
			.BUTTON_HOVER = "button hover",
			.BUTTON_FOCUS = "button focus",
			.BASE         = "base",
			.BASE_HOVER   = "base hover",
			.BASE_FOCUS   = "base focus",
			.SCROLL_BASE  = "scroll base",
			.SCROLL_THUMB = "scroll thumb",
			.SELECTION_BG = "selection bg",
		}

		sw := i32(f32(mu.get_current_container(ctx).body.w) * 0.14)
		mu.layout_row(ctx, {80, sw, sw, sw, sw, -1})
		for label, col in colors {
			mu.label(ctx, label)
			u8_slider(ctx, &ctx.style.colors[col].r, 0, 255)
			u8_slider(ctx, &ctx.style.colors[col].g, 0, 255)
			u8_slider(ctx, &ctx.style.colors[col].b, 0, 255)
			u8_slider(ctx, &ctx.style.colors[col].a, 0, 255)
			mu.draw_rect(ctx, mu.layout_next(ctx), ctx.style.colors[col])
		}
	}
}
