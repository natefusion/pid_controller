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
import "core:math/linalg"
import rl "vendor:raylib"

PIXEL_WINDOW_HEIGHT :: 180

Edit_Mode :: enum {
    None,
    Proportional,
    Integral,
    Derivative,
}

Particle :: struct {
    force : [3]f64,
    prop_spin_dir : f64,
    position_old : [3]f64,
    position : [3]f64,
    angular_position: [3]f64,
    angular_position_old: [3]f64,
    mass: f64,
    radius: f64,
}

Link :: struct {
    p: int,
    p1: int,
    length: f64,
}

Super_Particle :: [6]Link

PID_Controller :: struct {
    Kp: f64,
    Ki: f64,
    Kd: f64,
    setpoint: f64,
    A: [3]f64,
    error: [3]f64,
    output: f64,
}

Game_2D :: struct {
    particles: [2]Particle,
    level: f64,
    pid: PID_Controller,
    edit: Edit_Mode,
}

Game_3D :: struct {
    drone: Super_Particle,
    particles : [4]Particle,
    camera: rl.Camera3D,
    gyro : [3]f64,
    pid : [3]PID_Controller,
    edit: Edit_Mode,
}

Game_Memory :: struct {
	run: bool,
    g2d : Game_2D,
    g3d : Game_3D,
    dt: f64,
}

g: ^Game_Memory

set_game_3d_default :: proc(g: ^Game_3D) {
    g.camera = {
        position = {0.0, -4.0, 25.0},
        target = {0.0, 0.0, 0.0},
        up = {0.0, 0.0, 1.0},
        fovy = 90.0,
        projection = .PERSPECTIVE,
    }

    g.particles = {
        { // front right
            radius = 0.5,
            prop_spin_dir = 1,
            position = {1,1,20.5},
            position_old = {1,1,20.5},
            mass = 10,
        },
        { // front left
            radius = 0.5,
            prop_spin_dir = -1,
            position = {-1,1,20.5},
            position_old = {-1,1,20.5},
            mass = 10,
        },
        { // back right
            radius = 0.5,
            prop_spin_dir = 1,
            position = {1,-1,20},
            position_old = {1,-1,20},
            mass = 10,
        },
        { // back left
            radius = 0.5,
            prop_spin_dir = -1,
            position = {-1,-1,20},
            position_old = {-1,-1,20},
            mass = 10,
        },
    }

    g.drone = {
        {p = 0, p1 = 1, length = 2},
        {p = 0, p1 = 2, length = 2},
        {p = 1, p1 = 3, length = 2},
        {p = 3, p1 = 2, length = 2},
        
        {p = 0, p1 = 3, length = math.sqrt_f64(8.0)},
        {p = 1, p1 = 2, length = math.sqrt_f64(8.0)},
    }
}

// DrawTextCodepoint3D :: proc(font: rl.Font, codepoint: int, position: [3]f32, fontSize: f32, backface: bool, tint: rl.Color) {
//     position := position
//     // Character index position in sprite font
//     // NOTE: In case a codepoint is not available in the font, index returned points to '?'
//     index := rl.GetGlyphIndex(font, cast(rune)codepoint)
//     scale := fontSize / cast(f32)font.baseSize

//     // Character destination rectangle on screen
//     // NOTE: We consider charsPadding on drawing
//     position.x += cast(f32)(font.glyphs[index].offsetX - font.glyphPadding)*scale
//     position.z += cast(f32)(font.glyphs[index].offsetY - font.glyphPadding)*scale

//     // Character source rectangle from font texture atlas
//     // NOTE: We consider chars padding when drawing, it could be required for outline/glow shader effects
//     srcRec := rl.Rectangle{ font.recs[index].x - cast(f32)font.glyphPadding, font.recs[index].y - cast(f32)font.glyphPadding,
//                             font.recs[index].width + 2.0*cast(f32)font.glyphPadding, font.recs[index].height + 2.0*cast(f32)font.glyphPadding }

//     width := (font.recs[index].width + 2.0*cast(f32)font.glyphPadding)*scale
//     height := (font.recs[index].height + 2.0*cast(f32)font.glyphPadding)*scale

//     if (font.texture.id > 0) {
//         x := 0.0
//         y := 0.0
//         z := 0.0

//         // normalized texture coordinates of the glyph inside the font texture (0.0f -> 1.0f)
//         tx := srcRec.x/cast(f32)font.texture.width
//         ty := srcRec.y/cast(f32)font.texture.height
//         tw := (srcRec.x+srcRec.width)/cast(f32)font.texture.width
//         th := (srcRec.y+srcRec.height)/cast(f32)font.texture.height

//         SHOW_LETTER_BOUNDARY :: false
//         SHOW_TEXT_BOUNDARY :: false
//         LETTER_BOUNDRY_SIZE :: 0.25
//         TEXT_MAX_LAYERS :: 32
//         LETTER_BOUNDRY_COLOR :: rl.VIOLET

//         if SHOW_LETTER_BOUNDARY do rl.DrawCubeWiresV(rl.Vector3{ position.x + width/2, position.y, position.z + height/2}, rl.Vector3{ width, LETTER_BOUNDRY_SIZE, height }, LETTER_BOUNDRY_COLOR)

//         rl.CheckRenderBatchLimit(4 + 4*backface)
//         rl.rlSetTexture(font.texture.id)

//         rl.rlPushMatrix()
//         rlTranslatef(position.x, position.y, position.z)

//         rlBegin(RL_QUADS)
//         rlColor4ub(tint.r, tint.g, tint.b, tint.a)

//         // Front Face
//         rlNormal3f(0.0, 1.0, 0.0)                                   // Normal Pointing Up
//         rlTexCoord2f(tx, ty); rlVertex3f(x,         y, z)              // Top Left Of The Texture and Quad
//         rlTexCoord2f(tx, th); rlVertex3f(x,         y, z + height)     // Bottom Left Of The Texture and Quad
//         rlTexCoord2f(tw, th); rlVertex3f(x + width, y, z + height)     // Bottom Right Of The Texture and Quad
//         rlTexCoord2f(tw, ty); rlVertex3f(x + width, y, z)              // Top Right Of The Texture and Quad

//         if backface {
//             // Back Face
//             rlNormal3f(0.0, -1.0, 0.0)                              // Normal Pointing Down
//             rlTexCoord2f(tx, ty)
//             rlVertex3f(x,         y, z)          // Top Right Of The Texture and Quad
//             rlTexCoord2f(tw, ty)
//             rlVertex3f(x + width, y, z)          // Top Left Of The Texture and Quad
//             rlTexCoord2f(tw, th)
//             rlVertex3f(x + width, y, z + height) // Bottom Left Of The Texture and Quad
//             rlTexCoord2f(tx, th)
//             rlVertex3f(x,         y, z + height) // Bottom Right Of The Texture and Quad
//         }
//         rlEnd()
//         rlPopMatrix()

//         rlSetTexture(0)
//     }
// }

// // Draw a 2D text in 3D space
// DrawText3D :: proc(font: rl.Font, text: cstring, position: [3]f32, fontSize: f32, fontSpacing: f32, lineSpacing: f32, backface: bool, tint: rl.Color) {
//     length := rl.TextLength(text)          // Total length in bytes of the text, scanned by codepoints in loop

//     textOffsetY := 0.0               // Offset between lines (on line break '\n')
//     textOffsetX := 0.0               // Offset X to next character to draw

//     scale := fontSize/cast(f32)font.baseSize

//     for i := 0; i < length; {
//         // Get next codepoint from byte string and glyph index in font
//         codepointByteCount := 0
//         codepoint := rl.GetCodepoint(&text[i], &codepointByteCount)
//         index := rl.GetGlyphIndex(font, codepoint)

//         // NOTE: Normally we exit the decoding sequence as soon as a bad byte is found (and return 0x3f)
//         // but we need to draw all of the bad bytes using the '?' symbol moving one byte
//         if codepoint == 0x3f do codepointByteCount = 1

//         if codepoint == '\n' {
//             // NOTE: Fixed line spacing of 1.5 line-height
//             // TODO: Support custom line spacing defined by user
//             textOffsetY += fontSize + lineSpacing
//             textOffsetX = 0.0
//         } else {
//             if (codepoint != ' ') && (codepoint != '\t') {
//                 rl.DrawTextCodepoint3D(font, codepoint, (rl.Vector3){ position.x + textOffsetX, position.y, position.z + textOffsetY }, fontSize, backface, tint)
//             }

//             if font.glyphs[index].advanceX == 0 do textOffsetX += cast(f32)font.recs[index].width*scale + fontSpacing
//             else do textOffsetX += cast(f32)font.glyphs[index].advanceX*scale + fontSpacing
//         }

//         i += codepointByteCount   // Move text bytes counter to next codepoint
//     }
// }

set_angular_position :: proc(ps: []Particle, sp: ^Super_Particle) {
    ///////// AHH MAKE IT WORK IN 3D
    // ugleh
    // the location for the three particles in a super particle is specified in make_super_particle, at the bottom of the function
    // p1 : ^Particle = &ps[sp[0].p]
    // p2 : ^Particle = &ps[sp[1].p]
    // p3 : ^Particle = &ps[sp[1].p1]

    // a := sp[0].length
    // b := sp[1].length
    // c := sp[2].length
    // center := (a * p3.position + b * p1.position + c * p2.position) / (a + b + c)

    // straight := [3]f64 {center.x+1, center.y, center.z}

    // ref := straight-center
    // r1 := p1.position-center
    // r2 := p2.position-center
    // r3 := p3.position-center
    
    // temp := p1.angular_position
    // p1.angular_position = math.mod(math.TAU + linalg.angle_between(r1, ref) * math.sign(linalg.cross(r1, ref)), math.TAU)
    // p1.angular_position_old = temp

    // temp = p2.angular_position
    // p2.angular_position = math.mod(math.TAU + linalg.angle_between(r2, ref) * math.sign(linalg.cross(r2, ref)), math.TAU)
    // p2.angular_position_old = temp

    // temp = p3.angular_position
    // p3.angular_position = math.mod(math.TAU + linalg.angle_between(r3, ref) * math.sign(linalg.cross(r3, ref)), math.TAU)
    // p3.angular_position_old = temp
}

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

init_pid :: proc(using pid: ^PID_Controller) {
    Kp = 10.0
    Ki = 10.0
    Kd = 20.0
    setpoint = 0.0
    A = {
        Kp + Ki*g.dt + Kd/g.dt,
        -Kp - 2*Kd/g.dt,
        Kd/g.dt,
    }
    error = 0
    output = 0
}

reset_pid :: proc(using pid: ^PID_Controller) {
    error = 0
    output = 0
}

update_pid :: proc(measured_value: f64, pid: ^PID_Controller) {
    pid.A[0] = pid.Kp + pid.Ki*g.dt + pid.Kd/g.dt
    pid.A[1] = -pid.Kp - 2*pid.Kd/g.dt
    pid.A[2] = pid.Kd / g.dt
    
    pid.error[2] = pid.error[1]
    pid.error[1] = pid.error[0]
    pid.error[0] = pid.setpoint - measured_value
    pid.output += pid.A[0] * pid.error[0] + pid.A[1] * pid.error[1] + pid.A[2] * pid.error[2]
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

    end_y += 50

    return
}

update_particle :: proc(p: ^Particle) {
    temp := p.position
    a := p.force / p.mass
    x := a * g.dt * g.dt
    p.position = 2*p.position - p.position_old + x
    p.position_old = temp
    p.force = 0
}

gravity :: proc(p: ^Particle) {
    p.force += {0, 0, -10}
}

handle_ground_collision :: proc(p: ^Particle) {
    if p.position.y >= 700 {
        p.position.y = 700
        p.position_old.y = 700
    }
}

handle_ground_collision_3d :: proc(p: ^Particle) {
    if (p.position.z <= 0.1) {
        p.position.z = 0.1
        p.position_old.z = 0.1
    }
}

update_level :: proc() {
    a := g.g2d.particles[0].position
    b := g.g2d.particles[1].position
    g.g2d.level = linalg.atan2(abs(a.y - b.y), abs(a.x - b.x))
}

update_particles :: proc(particles: []Particle, two_d: bool = false) {
    for &p in particles {
        gravity(&p)
        update_particle(&p)
        if two_d {
            handle_ground_collision(&p)
        } else {
            handle_ground_collision_3d(&p)
        }
    }
}

lowest_particle :: proc() -> (lowest: int) {
    min_val := g.g2d.particles[0].position.y
    
    for p, i in g.g2d.particles {
        // yeah yeah backwards suck my fat one
        if p.position.y > min_val {
            min_val = p.position.y
            lowest = i
        }
    }
    return
}

lowest_particle_3d :: proc(g: ^Game_3D) -> (lowest, pitch_adj, roll_adj: int) {
    min_val := g.particles[0].position.z

    for p, i in g.particles {
        if p.position.z > min_val {
            min_val = p.position.z
            lowest = i
        }
    }

    // make sure this matches the super particle
    switch (lowest) {
    case 0:
        pitch_adj = 1
        roll_adj = 2
    case 1:
        pitch_adj = 0
        roll_adj = 3
    case 2:
        pitch_adj = 3
        roll_adj = 0
    case 3:
        pitch_adj = 2
        roll_adj = 1
    }

    return
}

handle_pid :: proc() {
    update_pid(g.g2d.level, &g.g2d.pid)
    force := g.g2d.pid.output
    i := lowest_particle()
    g.g2d.particles[i].force += {0, force, 0}
}

update_2d :: proc() {
    update_level()
    handle_pid()
    update_particles(g.g2d.particles[:], two_d=true)
}

draw_2d :: proc() {
	rl.BeginDrawing()
	rl.ClearBackground(rl.RAYWHITE)
    for p in g.g2d.particles {
        rl.DrawCircle(cast(i32)p.position.x, cast(i32)p.position.y, 10, rl.GREEN)
    }
    rl.DrawText(rl.TextFormat("%f radian", g.g2d.level), 10, 10, 20, rl.BLACK)
    draw_pid_stats(&g.g2d.pid)
    rl.DrawText(fmt.caprintf("edit mode: %v", g.g2d.edit), 10, 30, 20, rl.GREEN)
	rl.EndDrawing()
}

handle_input :: proc() {
    if (rl.IsKeyDown(.P)) {
        g.g2d.edit = .Proportional
    } else if (rl.IsKeyDown(.I)) {
        g.g2d.edit = .Integral
    } else if (rl.IsKeyDown(.D)) {
        g.g2d.edit = .Derivative
    } else if (rl.IsKeyDown(.N)) {
        g.g2d.edit = .None
    }

    mw := cast(f64)rl.GetMouseWheelMove() / 10.0

    switch g.g2d.edit {
    case .Proportional:
        g.g2d.pid.Kp += mw
    case .Integral:
        g.g2d.pid.Ki += mw
    case .Derivative:
        g.g2d.pid.Kd += mw
    case .None:
    }
}

handle_input_3d :: proc(g: ^Game_3D) {
    if rl.IsKeyDown(.P) {
        g.edit = .Proportional
    } else if rl.IsKeyDown(.I) {
        g.edit = .Integral
    } else if rl.IsKeyDown(.D) {
        g.edit = .Derivative
    } else if rl.IsKeyDown(.N) {
        g.edit = .None
    }

    mw : f64 = 0
    if rl.IsKeyDown(.LEFT_BRACKET) {
        mw -= 0.1
    }
    if rl.IsKeyDown(.RIGHT_BRACKET) {
        mw += 0.1
    }

    for &p in g.pid {
        switch g.edit {
        case .Proportional:
            p.Kp += mw
        case .Integral:
            p.Ki += mw
        case .Derivative:
            p.Kd += mw
        case .None:
        }
    }

    if rl.IsKeyPressed(.R) {
        for &p in g.pid do reset_pid(&p)
        set_game_3d_default(g)
    }

    
}

update_gyro :: proc(g: ^Game_3D) {
    a := g.particles[0].position
    n := drone_normal(g)
    c := drone_center(g)
    g.gyro[0] = linalg.angle_between(n.yz, c.yz)
    g.gyro[1] = linalg.angle_between(n.xz, c.xz)
    g.gyro[2] = linalg.atan2(a.x, a.y) - math.PI / 4
}

handle_pid3d :: proc(g: ^Game_3D) {
    update_pid(g.gyro[0], &g.pid[0])
    update_pid(g.gyro[1], &g.pid[1])
    update_pid(g.gyro[2], &g.pid[2])
    
    pitch_force := g.pid[0].output
    roll_force := g.pid[1].output
    i, pitch_adj, roll_adj := lowest_particle_3d(g)
    g.particles[i].force -= {0, 0, g.particles[i].prop_spin_dir * math.sqrt(math.pow(pitch_force, 2) + math.pow(roll_force, 2))}
    g.particles[pitch_adj].force -= {0, 0, g.particles[pitch_adj].prop_spin_dir * pitch_force}
    g.particles[roll_adj].force -= {0, 0, g.particles[roll_adj].prop_spin_dir * roll_force}

    yaw_force := g.pid[2].output
    for &p in g.particles {
        if p.prop_spin_dir > 0 {
            p.force -= yaw_force
        } else {
            p.force += yaw_force
        }
    }
}


update_3d :: proc() {
    rl.UpdateCamera(&g.g3d.camera, .FREE)
    update_gyro(&g.g3d)
    handle_pid3d(&g.g3d)
    update_particles(g.g3d.particles[:])
    for link in g.g3d.drone do update_link(g.g3d.particles[:], link)
    // set_angular_position(g.g3d.particles[:], &g.g3d.drone)
}

draw_drone :: proc() {
    for p in g.g3d.particles {
        rl.DrawSphere({cast(f32)p.position.x, cast(f32)p.position.y, cast(f32)p.position.z}, cast(f32)p.radius, rl.GREEN)
    }
}

draw_gyro :: proc(gyro: [3]f64) {
    gyro_deg := gyro * 180 / math.PI
    rl.DrawText(fmt.caprintf("Pitch: %v\nRoll:   %v\nYaw:   %v", gyro_deg.x, gyro_deg.y, gyro_deg.z), 10, 10, 20, rl.BLACK)
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
    front := cast_f32(drone_front(g))
    rl.DrawSphere(c + normal, 0.125, rl.RED)
    rl.DrawSphere(c, 0.125, rl.PURPLE)
    rl.DrawSphere(front, 0.125, rl.YELLOW)
}

draw_3d :: proc(g: ^Game_3D) {
    rl.BeginDrawing()
    rl.ClearBackground(rl.RAYWHITE)
    draw_gyro(g.gyro)
    end_y : i32 = 100
    end_y = draw_pid_stats(&g.pid[0], end_y)
    end_y = draw_pid_stats(&g.pid[1], end_y)
    end_y = draw_pid_stats(&g.pid[2], end_y)
    rl.DrawText(fmt.caprintf("edit mode: %v", g.edit), 10, end_y, 20, rl.GREEN)
    rl.BeginMode3D(g.camera)
    draw_drone()
    rl.DrawGrid(10, 2.0)
    draw_normal(g)
    draw_links(g)
    rl.EndMode3D()
    rl.EndDrawing()
}

game_2d :: proc() {
    update_2d()
    draw_2d()
    handle_input()
}

game_3d :: proc() {
    update_3d()
    draw_3d(&g.g3d)
    handle_input_3d(&g.g3d)
}

@(export)
game_update :: proc() {
    // game_2d()
    game_3d()

	// Everything on tracking allocator is valid until end-of-frame.
	free_all(context.temp_allocator)
}

@(export)
game_init_window :: proc() {
	rl.SetConfigFlags({.WINDOW_RESIZABLE, .VSYNC_HINT})
	rl.InitWindow(1280, 720, "Pid Controller")
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
        g2d = {
            particles = {
                {
                    radius = 10,
                    position = apos,
                    position_old = apos,
                    mass = 10,
                },
                {
                    radius = 10,
                    position = bpos,
                    position_old = bpos,
                    mass = 10,
                },
            },
            edit = .None,
        },
        dt = 0.001,
	}

    set_game_3d_default(&g.g3d)

    init_pid(&g.g2d.pid)

    for &p in g.g3d.pid {
        init_pid(&p)
    }
    

    g.g3d.pid[2].setpoint = 0.2

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
