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
// import "core:math"
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
    force : [2]f64,
    position_old : [2]f64,
    position : [2]f64,
    mass: f64,
    radius: f64,
}

PID_Controller :: struct {
    Kp: f64,
    Ki: f64,
    Kd: f64,
    setpoint: f64,
    A: [3]f64,
    error: [3]f64,
    output: f64,
}

Game_Memory :: struct {
	run: bool,
    particles: [2]Particle,
    level: f64,
    pid: PID_Controller,
    edit: Edit_Mode,
    dt: f64,
}

g: ^Game_Memory

init_pid :: proc(using pid: ^PID_Controller) {
    Kp = 1000.0
    Ki = 1000.0
    Kd = 1000.0
    setpoint = 0.0
    A = {
        Kp + Ki*g.dt + Kd/g.dt,
        -Kp - 2*Kd/g.dt,
        Kd/g.dt,
    }
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

draw_pid_stats :: proc() {

    str := fmt.caprintf("PID OUT: %f", g.pid.output)
    rl.DrawText(str, 200, 10, 20, rl.RED)

    str = fmt.caprintf("PID ERR: %f", g.pid.error)
    rl.DrawText(str, 200, 30, 20, rl.BLACK)

    str = fmt.caprintf("Kp: %f, Ki: %f, Kd: %f", g.pid.Kp, g.pid.Ki, g.pid.Kd)
    rl.DrawText(str, 200, 50, 20, rl.BLACK)
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
    p.force += {0, 1000}
}

handle_ground_collision :: proc(p: ^Particle) {
    if p.position.y >= 700 {
        p.position.y = 700
        p.position_old.y = 700
    }
}

update_level :: proc() {
    a := g.particles[0].position
    b := g.particles[1].position
    g.level = linalg.atan2(abs(a.y - b.y), abs(a.x - b.x))
}

update_particles :: proc() {
    for &p in g.particles {
        gravity(&p)
        update_particle(&p)
        handle_ground_collision(&p)
    }
}

lowest_particle :: proc() -> (lowest: int) {
    min_val := g.particles[0].position.y
    
    for p, i in g.particles {
        // yeah yeah backwards suck my fat one
        if p.position.y > min_val {
            min_val = p.position.y
            lowest = i
        }
    }
    return
}

handle_pid :: proc() {
    update_pid(g.level, &g.pid)
    force := g.pid.output
    i := lowest_particle()
    g.particles[i].force += {0, force}
}

update :: proc() {
    update_level()
    handle_pid()
    update_particles()
}

draw :: proc() {
	rl.BeginDrawing()
	rl.ClearBackground(rl.RAYWHITE)
    for p in g.particles {
        rl.DrawCircle(cast(i32)p.position.x, cast(i32)p.position.y, 10, rl.GREEN)
    }
    rl.DrawText(rl.TextFormat("%f radian", g.level), 10, 10, 20, rl.BLACK)
    draw_pid_stats()
    rl.DrawText(fmt.caprintf("edit mode: %v", g.edit), 10, 30, 20, rl.GREEN)
	rl.EndDrawing()
}

handle_input :: proc() {
    if (rl.IsKeyDown(.P)) {
        g.edit = .Proportional
    } else if (rl.IsKeyDown(.I)) {
        g.edit = .Integral
    } else if (rl.IsKeyDown(.D)) {
        g.edit = .Derivative
    } else if (rl.IsKeyDown(.N)) {
        g.edit = .None
    }

    mw := cast(f64)rl.GetMouseWheelMove() / 10.0

    switch g.edit {
    case .Proportional:
        g.pid.Kp += mw
    case .Integral:
        g.pid.Ki += mw
    case .Derivative:
        g.pid.Kd += mw
    case .None:
    }
}

@(export)
game_update :: proc() {
    update()
	draw()
    handle_input()

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
}

@(export)
game_init :: proc() {
	g = new(Game_Memory)

    apos := [2]f64{100, 100}
    bpos := [2]f64{150, 120}
	g^ = Game_Memory {
		run = true,
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
        dt = 0.001,
        edit = .None,
	}

    init_pid(&g.pid)

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
