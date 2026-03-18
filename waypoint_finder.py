import glob
import json
import os
import sys

import carla
import pygame

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.tools.scenario_helper import get_opposite_dir_lanes, get_same_dir_lanes

_HOST_ = "127.0.0.1"
_PORT_ = 2000
_CONFIG_FILE_ = "waypoint_finder_config.json"


def load_config():
    """Load configuration from file."""
    default_config = {
        "lane_type_index": 0,
        "save_counter": 0,
        "export_format": "python",  # or "openscenario"
        "copy_to_clipboard": True,
        "window_width": 700,
        "window_height": 650,
        "host": _HOST_,
        "port": _PORT_,
    }
    try:
        if os.path.exists(_CONFIG_FILE_):
            with open(_CONFIG_FILE_, "r") as f:
                config = json.load(f)
                # Merge with defaults to handle new config keys
                return {**default_config, **config}
    except Exception as e:
        print(f"Error loading config: {e}")
    return default_config


def save_config(config):
    """Save configuration to file."""
    try:
        with open(_CONFIG_FILE_, "w") as f:
            json.dump(config, f, indent=2)
    except Exception as e:
        print(f"Error saving config: {e}")


def draw_text(screen, text, pos, font, color=(255, 255, 255)):
    text_surface = font.render(text, True, color)
    screen.blit(text_surface, pos)


def draw_point(world, location, color):
    world.debug.draw_string(
        location,
        "O",
        draw_shadow=False,
        color=color,
        life_time=1.0,
        persistent_lines=True,
    )


def draw_waypoint(world, waypoint, color):
    draw_point(world, waypoint.transform.location, color)


def draw_transform_with_direction(world, transform, color, distance=2.0):
    """Draw a transform location with its forward vector direction marker."""
    loc = transform.location
    draw_point(world, loc, color)

    forward_vec = transform.get_forward_vector()
    direction_location = loc + carla.Location(
        x=forward_vec.x * distance,
        y=forward_vec.y * distance,
        z=forward_vec.z * distance,
    )

    world.debug.draw_string(
        direction_location,
        "X",
        draw_shadow=False,
        color=color,
        life_time=1.0,
        persistent_lines=True,
    )

    world.debug.draw_line(
        loc,
        direction_location,
        thickness=0.1,
        color=color,
        life_time=1.0,
        persistent_lines=True,
    )


def draw_waypoint_with_direction(world, waypoint, color, distance=2.0):
    """Draw waypoint with its forward vector direction marker."""
    draw_transform_with_direction(world, waypoint.transform, color, distance)


def format_transform_python(transform, name=""):
    """Format a carla.Transform as Python code."""
    loc = transform.location
    rot = transform.rotation
    return (
        f"carla.Transform(carla.Location(x={loc.x:.2f}, y={loc.y:.2f}, z={loc.z:.2f}), "
        f"carla.Rotation(pitch={rot.pitch:.2f}, yaw={rot.yaw:.2f}, roll={rot.roll:.2f})),"
    )


def format_transform_openscenario(transform, name=""):
    """Format a carla.Transform as OpenScenario WorldPosition XML."""
    loc = transform.location
    rot = transform.rotation
    return (
        f'<WorldPosition x="{loc.x:.2f}" y="{loc.y:.2f}" '
        f'z="{loc.z:.2f}" h="{rot.yaw:.2f}" '
        f'p="{rot.pitch:.2f}" r="{rot.roll:.2f}"/>'
    )


def copy_text_to_clipboard(text):
    """Copy text to system clipboard using tkinter."""
    try:
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
        root.clipboard_clear()
        root.clipboard_append(text)
        root.update()
        root.destroy()
    except Exception as e:
        print(f"Could not copy to clipboard: {e}")


def raycast_down(world, location, max_distance=500.0):
    """Cast a ray straight down from location and return first hit location, or None."""
    ray_end = carla.Location(x=location.x, y=location.y, z=location.z - max_distance)
    hits = world.cast_ray(location, ray_end)
    if hits:
        return hits[0].location
    return None


def main():
    # Load configuration
    config = load_config()

    # Initialize Pygame
    pygame.init()
    screen = pygame.display.set_mode((config["window_width"], config["window_height"]))
    pygame.display.set_caption("CARLA Waypoint Saver")
    font = pygame.font.Font(None, 28)
    clock = pygame.time.Clock()

    # Button settings
    button_color = (70, 130, 180)
    button_hover_color = (100, 160, 210)
    button_rect = pygame.Rect(20, 470, 200, 50)

    # Input box settings
    input_box = pygame.Rect(20, 470, 420, 40)
    input_active = False
    wp_name = ""

    # Connect to CARLA
    client = carla.Client(_HOST_, _PORT_)
    client.set_timeout(60.0)
    print(f"Connecting to CARLA at {_HOST_}:{_PORT_} ...")
    for attempt in range(10):
        try:
            world = client.get_world()
            break
        except Exception as e:
            print(f"  Attempt {attempt + 1}/10 failed: {e}. Retrying...")
            pygame.time.wait(2000)
    else:
        print("Could not connect to CARLA after 10 attempts. Is the simulator running?")
        pygame.quit()
        return
    carla_map = world.get_map()

    data_provider = CarlaDataProvider()

    running = True
    saved_msg_timer = 0
    last_save_time = 0
    save_cooldown_ms = 500  # ms before another save is allowed

    save_counter = config["save_counter"]
    wp_name = f"Waypoint_{save_counter}"

    # Lane type settings — None signals "Raw" raycast mode
    lane_types = [
        ("Driving", carla.LaneType.Driving),
        ("Sidewalk", carla.LaneType.Sidewalk),
        ("Shoulder", carla.LaneType.Shoulder),
        ("Raw", None),
    ]

    current_lane_type_index = config["lane_type_index"]
    export_format = config["export_format"]  # "python" or "openscenario"
    copy_to_clipboard = config["copy_to_clipboard"]

    # Clipboard checkbox settings
    checkbox_rect = pygame.Rect(20, 435, 18, 18)

    # saved_waypoints entries: {name, transform, waypoint (or None), is_raw, index}
    saved_waypoints = []

    # UI list settings
    list_panel_x = 450
    list_panel_y = 20
    list_panel_width = 230
    list_panel_height = 420
    list_item_height = 60
    visible_items = int(list_panel_height / list_item_height)
    del_btn_width = 22  # width of the delete "X" button on each list item
    scroll_offset = 0
    last_click_time = 0
    last_click_index = -1

    while running:
        screen.fill((30, 30, 30))
        mouse_pos = pygame.mouse.get_pos()
        spectator = world.get_spectator()
        transform = spectator.get_transform()
        location = transform.location
        rotation = transform.rotation

        is_raw = lane_types[current_lane_type_index][1] is None

        # Resolve the current effective transform and waypoint
        current_waypoint = None
        if is_raw:
            hit = raycast_down(world, location)
            if hit is not None:
                # Use only yaw from the spectator — pitch/roll reflect camera tilt, not road heading
                current_effective_transform = carla.Transform(
                    hit, carla.Rotation(pitch=0.0, yaw=rotation.yaw, roll=0.0)
                )
            else:
                current_effective_transform = transform  # fallback: spectator itself
        else:
            current_waypoint = carla_map.get_waypoint(
                location, lane_type=lane_types[current_lane_type_index][1]
            )
            current_effective_transform = current_waypoint.transform

        # Draw current point (red) with direction
        draw_transform_with_direction(world, current_effective_transform, carla.Color(255, 0, 0))

        # Draw all saved points (yellow) with direction and sequential number label
        for seq_num, saved_wp_data in enumerate(saved_waypoints, start=1):
            draw_transform_with_direction(world, saved_wp_data["transform"], carla.Color(255, 255, 0))
            label_loc = saved_wp_data["transform"].location + carla.Location(z=1.5)
            world.debug.draw_string(
                label_loc,
                str(seq_num),
                draw_shadow=False,
                color=carla.Color(255, 220, 0),
                life_time=1.0,
                persistent_lines=True,
            )

        # Draw road preview (next/prev waypoints) — only in waypoint modes
        if not is_raw and current_waypoint is not None:
            front_waypoint = current_waypoint.next(1)
            for i in range(5):
                try:
                    front_waypoint = front_waypoint[0].next(1)
                    draw_waypoint(world, front_waypoint[0], carla.Color(0, 255, 0))
                except IndexError:
                    break

            prev_waypoint = current_waypoint.previous(1)
            for i in range(5):
                try:
                    prev_waypoint = prev_waypoint[0].previous(1)
                    draw_waypoint(world, prev_waypoint[0], carla.Color(0, 255, 0))
                except IndexError:
                    break

        def do_save():
            nonlocal save_counter, wp_name, saved_msg_timer, last_save_time
            now = pygame.time.get_ticks()
            if now - last_save_time < save_cooldown_ms:
                return
            if not wp_name.strip():
                return
            if export_format == "python":
                entry = format_transform_python(current_effective_transform, wp_name) + "\n"
            else:
                entry = format_transform_openscenario(current_effective_transform, wp_name) + "\n"
            print(entry.strip())
            if copy_to_clipboard:
                copy_text_to_clipboard(entry.strip())
            saved_waypoints.append({
                "name": wp_name,
                "transform": current_effective_transform,
                "waypoint": current_waypoint,
                "is_raw": is_raw,
                "index": save_counter,
            })
            save_counter += 1
            wp_name = f"Waypoint_{save_counter}"
            saved_msg_timer = 60
            last_save_time = now
            config["save_counter"] = save_counter
            save_config(config)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Mouse wheel scrolling for waypoint list
            elif event.type == pygame.MOUSEWHEEL:
                if list_panel_x <= mouse_pos[0] <= list_panel_x + list_panel_width:
                    scroll_offset = max(0, min(scroll_offset - event.y, len(saved_waypoints) - int(list_panel_height / list_item_height)))

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if checkbox_rect.collidepoint(event.pos):
                    copy_to_clipboard = not copy_to_clipboard
                    config["copy_to_clipboard"] = copy_to_clipboard
                    save_config(config)

                # Check if click is in waypoint list
                if (list_panel_x <= event.pos[0] <= list_panel_x + list_panel_width and
                        list_panel_y <= event.pos[1] <= list_panel_y + list_panel_height):
                    relative_y = event.pos[1] - list_panel_y
                    clicked_index = int(relative_y / list_item_height) + scroll_offset
                    if 0 <= clicked_index < len(saved_waypoints):
                        # Delete button zone — rightmost del_btn_width px of the item
                        if event.pos[0] >= list_panel_x + list_panel_width - del_btn_width:
                            saved_waypoints.pop(clicked_index)
                            scroll_offset = min(scroll_offset, max(0, len(saved_waypoints) - visible_items))
                            last_click_index = -1
                        else:
                            current_time = pygame.time.get_ticks()
                            if (clicked_index == last_click_index and
                                    current_time - last_click_time < 500):
                                # Double-click — teleport spectator above saved point
                                wp_data = saved_waypoints[clicked_index]
                                wp_loc = wp_data["transform"].location
                                teleport_location = carla.Location(
                                    x=wp_loc.x, y=wp_loc.y, z=wp_loc.z + 15.0
                                )
                                spectator.set_transform(carla.Transform(
                                    teleport_location,
                                    carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)
                                ))
                                print(f"Teleported to {wp_data['name']}")
                            last_click_time = current_time
                            last_click_index = clicked_index
                    input_active = False
                elif input_box.collidepoint(event.pos):
                    input_active = True
                else:
                    input_active = False

                if button_rect.collidepoint(event.pos):
                    do_save()

            elif event.type == pygame.KEYDOWN:
                if input_active:
                    if event.key == pygame.K_RETURN:
                        input_active = False
                    elif event.key == pygame.K_BACKSPACE:
                        wp_name = wp_name[:-1]
                    else:
                        wp_name += event.unicode
                else:
                    if event.key == pygame.K_TAB:
                        current_lane_type_index = (current_lane_type_index + 1) % len(lane_types)
                        config["lane_type_index"] = current_lane_type_index
                        save_config(config)
                    elif event.key == pygame.K_s:
                        do_save()
                    elif event.key == pygame.K_e:
                        export_format = "openscenario" if export_format == "python" else "python"
                        config["export_format"] = export_format
                        save_config(config)
                    elif event.key == pygame.K_c:
                        copy_to_clipboard = not copy_to_clipboard
                        config["copy_to_clipboard"] = copy_to_clipboard
                        save_config(config)

        # --- UI rendering ---

        draw_text(screen, "Spectator Location:", (20, 20), font)
        draw_text(screen, f"X: {location.x:.2f}  Y: {location.y:.2f}  Z: {location.z:.2f}", (20, 50), font)
        draw_text(screen, f"Rotation: Pitch: {rotation.pitch:.2f}  Yaw: {rotation.yaw:.2f}  Roll: {rotation.roll:.2f}", (20, 80), font)

        lane_type_color = (255, 100, 100) if is_raw else (100, 255, 100)
        draw_text(
            screen,
            f"Lane Type: {lane_types[current_lane_type_index][0]} (TAB to cycle)",
            (20, 110),
            font,
            color=lane_type_color,
        )
        draw_text(
            screen,
            f"Export Format: {export_format.upper()} (E to toggle)",
            (20, 140),
            font,
            color=(255, 200, 100),
        )

        if is_raw:
            eff_loc = current_effective_transform.location
            draw_text(screen, "Raw Hit Location:", (20, 170), font)
            draw_text(screen, f"X: {eff_loc.x:.2f}  Y: {eff_loc.y:.2f}  Z: {eff_loc.z:.2f}", (20, 200), font)
            draw_text(screen, "(Raycast hit — no road/lane data)", (20, 230), font, color=(180, 180, 180))
        else:
            draw_text(screen, "Closest Waypoint:", (20, 170), font)
            draw_text(screen, f"Road ID: {current_waypoint.road_id}", (20, 200), font)
            draw_text(screen, f"Lane ID: {current_waypoint.lane_id}", (20, 230), font)
            draw_text(screen, f"Lane Change: {str(current_waypoint.lane_change)}", (20, 260), font)
            draw_text(screen, f"Section ID: {current_waypoint.section_id}", (20, 290), font)
            draw_text(screen, f"N lanes: {len(get_same_dir_lanes(current_waypoint))}", (20, 320), font)
            draw_text(screen, f"N opposite lanes: {len(get_opposite_dir_lanes(current_waypoint))}", (20, 350), font)
            draw_text(
                screen,
                f"Distance to intersection: {current_waypoint.transform.location.distance(current_waypoint.next_until_lane_end(1)[-1].transform.location):.2f}m",
                (20, 380),
                font,
            )

        draw_text(screen, f"Saved waypoints: {len(saved_waypoints)}", (20, 410), font, color=(255, 255, 0))

        # Clipboard checkbox
        pygame.draw.rect(screen, (200, 200, 200), checkbox_rect, 2)
        if copy_to_clipboard:
            pygame.draw.rect(screen, (100, 220, 100), checkbox_rect.inflate(-4, -4))
        draw_text(screen, "Copy to clipboard (C)", (checkbox_rect.right + 8, checkbox_rect.y), font, color=(200, 200, 200))

        # Input box
        pygame.draw.rect(screen, (255, 255, 255), input_box, 2)
        draw_text(screen, "WP Name:", (input_box.x, input_box.y - 25), font)
        txt_surface = font.render(wp_name, True, (255, 255, 255))
        screen.blit(txt_surface, (input_box.x + 5, input_box.y + 5))

        # Save button
        pygame.draw.rect(
            screen,
            button_hover_color if button_rect.collidepoint(mouse_pos) else button_color,
            button_rect,
        )
        draw_text(screen, "Save Waypoint", (button_rect.x + 20, button_rect.y + 15), font)

        if saved_msg_timer > 0:
            draw_text(screen, "Waypoint saved!", (250, 455), font, color=(0, 255, 0))
            saved_msg_timer -= 1

        # Waypoint list panel
        pygame.draw.rect(screen, (50, 50, 50), (list_panel_x, list_panel_y, list_panel_width, list_panel_height))
        pygame.draw.rect(screen, (100, 100, 100), (list_panel_x, list_panel_y, list_panel_width, list_panel_height), 2)

        list_font = pygame.font.Font(None, 20)
        draw_text(screen, "Saved Waypoints (double-click to teleport):", (list_panel_x + 5, list_panel_y - 20), list_font, color=(200, 200, 200))

        for i in range(visible_items):
            wp_index = i + scroll_offset
            if wp_index >= len(saved_waypoints):
                break

            wp_data = saved_waypoints[wp_index]
            item_y = list_panel_y + i * list_item_height

            item_color = (60, 60, 60) if wp_index % 2 == 0 else (70, 70, 70)
            pygame.draw.rect(screen, item_color, (list_panel_x + 2, item_y + 2, list_panel_width - 4, list_item_height - 4))

            wp_loc = wp_data["transform"].location
            draw_text(screen, f"#{wp_index + 1}: {wp_data['name']}", (list_panel_x + 5, item_y + 5), list_font, color=(255, 255, 100))
            draw_text(screen, f"X:{wp_loc.x:.1f} Y:{wp_loc.y:.1f}", (list_panel_x + 5, item_y + 25), list_font, color=(200, 200, 200))
            if wp_data["is_raw"]:
                draw_text(screen, "Raw", (list_panel_x + 5, item_y + 43), list_font, color=(255, 100, 100))
            else:
                draw_text(screen, f"Road:{wp_data['waypoint'].road_id} Lane:{wp_data['waypoint'].lane_id}", (list_panel_x + 5, item_y + 43), list_font, color=(150, 150, 150))

            # Delete button ("X") on the right side of the item
            del_btn_x = list_panel_x + list_panel_width - del_btn_width - 1
            del_btn_y = item_y + (list_item_height - del_btn_width) // 2
            del_hover = (del_btn_x <= mouse_pos[0] <= del_btn_x + del_btn_width and
                         del_btn_y <= mouse_pos[1] <= del_btn_y + del_btn_width)
            del_color = (220, 80, 80) if del_hover else (160, 50, 50)
            pygame.draw.rect(screen, del_color, (del_btn_x, del_btn_y, del_btn_width, del_btn_width), border_radius=3)
            draw_text(screen, "X", (del_btn_x + 5, del_btn_y + 3), list_font, color=(255, 255, 255))

        # Keyboard shortcuts help
        help_font = pygame.font.Font(None, 22)
        draw_text(screen, "Keyboard Shortcuts:", (20, 540), help_font, color=(200, 200, 200))
        draw_text(screen, "TAB: Cycle lane types  |  S: Save  |  E: Toggle format  |  C: Toggle clipboard", (20, 565), help_font, color=(150, 150, 150))

        pygame.display.flip()
        clock.tick(30)

    # Save config before exiting
    config["save_counter"] = save_counter
    config["lane_type_index"] = current_lane_type_index
    config["export_format"] = export_format
    config["copy_to_clipboard"] = copy_to_clipboard
    save_config(config)

    pygame.quit()


if __name__ == "__main__":
    main()
