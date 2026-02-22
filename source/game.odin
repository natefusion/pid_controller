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

import "core:fmt"
import "core:math"
import "core:thread"
import "core:math/linalg"
import "core:math/cmplx"
import rl "vendor:raylib"


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
        game.throttle = clamp(game.throttle, 0, 1);
    } else if rl.IsKeyPressed(.MINUS) {
        game.throttle -= 0.1
        game.throttle = clamp(game.throttle, 0, 1);
    }

    if rl.IsKeyPressed(.SEMICOLON) {
        game.pid[2].setpoint += 0.1 * math.PI
        game.pid[2].setpoint = clamp(game.pid[2].setpoint, -math.PI, math.PI);
    } else if rl.IsKeyPressed(.APOSTROPHE) {
        game.pid[2].setpoint -= 0.1 * math.PI
        game.pid[2].setpoint = clamp(game.pid[2].setpoint, -math.PI, math.PI);
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
        // raylib quirk; target and position x,y cannot match exactly!!
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
    n := drone_normal(g)
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

clamp_motor_speed :: proc(a, b: ^f64) {
    compare :: proc(x, y: f64) -> (x1, y1: f64) {
        if x > 1 {
            extra := x - 1
            x1 = 1
            y1 = y - extra
            if y1 < 0 do y1 = 0
        } else if x < 0 {
            extra := math.abs(x)
            x1 = 0
            y1 = y + extra
            if y1 > 1 do y1 = 1
        } else {
            x1 = x
            y1 = y
        }
        return
    }

    a1, b1 := compare(a^, b^)
    b2, a2 := compare(b1, a1)
    assert(a2 <= 1 && a2 >= 0)
    assert(b2 <= 1 && b2 >= 0)

    a^ = a2
    b^ = b2
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
    rl.DrawRectangleRoundedLinesEx(bounds, 0.0, 0, 1, rl.BLACK);
    rl.DrawText(name, i32(bounds.x) + 2, i32(bounds.y) + 2, 10, rl.GRAY);
    
    h_offset := bounds.y + bounds.height;
    h_ratio := bounds.height / 4.0;

    posval := f32(pos) * bounds.width / f32(len(data))
    
    pv := [2]f32{0, 0};
    v := [2]f32{bounds.x, clamp(h_offset - h_ratio*(f32(data[0]) + h_shift), bounds.y, h_offset) };
    for i in 0..<bounds.width {
        idx := int(f32(i) * f32(len(data)) / bounds.width)
        y := f32(data[idx]) + h_shift
        pv := v
        v = [2]f32{ bounds.x + i, clamp(h_offset - h_ratio*y, bounds.y, h_offset) };
        rl.DrawLineEx(pv, v, 2, i < posval ? rl.RED : rl.Color {255,0,0,32});
    }
    
    rl.DrawLineEx({bounds.x + posval, bounds.y}, {bounds.x + posval, bounds.y + bounds.height}, 1, rl.GREEN)

    yval := clamp(h_offset - h_ratio*(f32(setpoint) + h_shift), bounds.y, h_offset)
    rl.DrawLineEx({bounds.x, yval }, {bounds.x + bounds.width, yval}, 1, rl.BLUE)
}


draw_3d :: proc(g: ^Game_3D) {
    rl.BeginDrawing()
    rl.ClearBackground(rl.RAYWHITE)
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

        DrawPlotSimple(rec, fmt.caprint(Pid_Type(i)), p.data[:], g.pid[i].setpoint, p.data_idx);
    }


    rl.EndDrawing()
}

update_3d :: proc(g: ^Game_3D) {
    update_gyro(g)
    handle_pid3d(g)
    update_particles(g.particles[:], g.dt)
    for link in g.drone do update_link(g.particles[:], link)
}

game_3d :: proc(game: ^Game_3D) {
    if !game.paused {
        update_3d(game)
    }

    if (game.view_mode == .Follow) {
        game.camera = {
            position = cast_f32(drone_center(game)) + 10,
            target = cast_f32(drone_center(game)),
            up = {0.0, 0.0, 1.0},
            fovy = 90.0,
            projection = .PERSPECTIVE,
        }
    }
    rl.UpdateCamera(&game.camera, .FREE)
    
    draw_3d(game)
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

@(export)
game_init_window :: proc() {
	rl.SetConfigFlags({.WINDOW_RESIZABLE, .VSYNC_HINT})
	rl.InitWindow(WIDTH, HEIGHT, "Pid Controller")
	rl.SetWindowPosition(200, 200)
	rl.SetTargetFPS(60)
	rl.SetExitKey(nil)
    rl.HideCursor()
    rl.DisableCursor()
}

@(export)
game_init :: proc() {
	g = new(Game_Memory)

    apos := [3]f64{100, 100, 0}
    bpos := [3]f64{150, 120, 0}
	g^ = Game_Memory {
		run = true,
        running_thread = false,
	}

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
