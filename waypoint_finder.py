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


def draw_waypoint(world, waypoint, color):
    # Draw the waypoint in the world
    world.debug.draw_string(
        waypoint.transform.location,
        "O",
        draw_shadow=False,
        color=color,
        life_time=1.0,
        persistent_lines=True,
    )


def draw_waypoint_with_direction(world, waypoint, color, distance=2.0):
    """Draw waypoint with its forward vector direction marker."""
    # Draw the waypoint location
    draw_waypoint(world, waypoint, color)

    # Calculate forward vector position
    forward_vec = waypoint.transform.get_forward_vector()
    direction_location = waypoint.transform.location + carla.Location(
        x=forward_vec.x * distance,
        y=forward_vec.y * distance,
        z=forward_vec.z * distance
    )

    # Draw the direction marker as "X"
    world.debug.draw_string(
        direction_location,
        "X",
        draw_shadow=False,
        color=color,
        life_time=1.0,
        persistent_lines=True,
    )

    # Draw line connecting waypoint to direction marker
    world.debug.draw_line(
        waypoint.transform.location,
        direction_location,
        thickness=0.1,
        color=color,
        life_time=1.0,
        persistent_lines=True,
    )


def format_waypoint_python(waypoint, name=""):
    """Format waypoint as Python carla.Transform."""
    return (
        f"carla.Transform(carla.Location(x={waypoint.transform.location.x:.2f}, y={waypoint.transform.location.y:.2f}, z={waypoint.transform.location.z:.2f}), "
        f"carla.Rotation(pitch={waypoint.transform.rotation.pitch:.2f}, yaw={waypoint.transform.rotation.yaw:.2f}, roll={waypoint.transform.rotation.roll:.2f})),"
    )


def format_waypoint_openscenario(waypoint, name=""):
    """Format waypoint as OpenScenario WorldPosition XML."""
    return (
        f'<WorldPosition x="{waypoint.transform.location.x:.2f}" y="{waypoint.transform.location.y:.2f}" '
        f'z="{waypoint.transform.location.z:.2f}" h="{waypoint.transform.rotation.yaw:.2f}" '
        f'p="{waypoint.transform.rotation.pitch:.2f}" r="{waypoint.transform.rotation.roll:.2f}"/>'
    )


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
    client.set_timeout(2.0)
    world = client.get_world()
    carla_map = world.get_map()

    data_provider = CarlaDataProvider()

    running = True
    saved_msg_timer = 0

    save_counter = config["save_counter"]
    wp_name = f"Waypoint_{save_counter}"

    # Lane type settings
    lane_types = [
        ("Driving", carla.LaneType.Driving),
        ("Sidewalk", carla.LaneType.Sidewalk),
        ("Shoulder", carla.LaneType.Shoulder),
    ]
    
    current_lane_type_index = config["lane_type_index"]
    export_format = config["export_format"]  # "python" or "openscenario"

    # Track saved waypoints to draw them in the world
    saved_waypoints = []  # List of dicts: {"name": str, "waypoint": carla.Waypoint, "index": int}

    # UI list settings
    list_panel_x = 450
    list_panel_y = 20
    list_panel_width = 230
    list_panel_height = 420
    list_item_height = 60
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

        # Get waypoint based on current lane type
        current_waypoint = carla_map.get_waypoint(
            location, lane_type=lane_types[current_lane_type_index][1]
        )

        # Draw the current waypoint in the world (red) with direction
        draw_waypoint_with_direction(world, current_waypoint, carla.Color(255, 0, 0))

        # Draw all saved waypoints (yellow) with direction
        for saved_wp_data in saved_waypoints:
            draw_waypoint_with_direction(world, saved_wp_data["waypoint"], carla.Color(255, 255, 0))

        front_waypoint = current_waypoint
        front_waypoint = front_waypoint.next(1)
        for i in range(5):
            try:
                front_waypoint = front_waypoint[0].next(1)
                draw_waypoint(world, front_waypoint[0], carla.Color(0, 255, 0))
            except IndexError:
                break

        prev_waypoint = current_waypoint
        prev_waypoint = prev_waypoint.previous(1)
        for i in range(5):
            try:
                prev_waypoint = prev_waypoint[0].previous(1)
                draw_waypoint(world, prev_waypoint[0], carla.Color(0, 255, 0))
            except IndexError:
                break
        # Draw the waypoint in the world

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Mouse wheel scrolling for waypoint list
            elif event.type == pygame.MOUSEWHEEL:
                if mouse_pos[0] >= list_panel_x and mouse_pos[0] <= list_panel_x + list_panel_width:
                    scroll_offset = max(0, min(scroll_offset - event.y, len(saved_waypoints) - int(list_panel_height / list_item_height)))

            # Click in input box or waypoint list
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Check if click is in waypoint list
                if (list_panel_x <= event.pos[0] <= list_panel_x + list_panel_width and
                    list_panel_y <= event.pos[1] <= list_panel_y + list_panel_height):
                    # Calculate which item was clicked
                    relative_y = event.pos[1] - list_panel_y
                    clicked_index = int(relative_y / list_item_height) + scroll_offset

                    if 0 <= clicked_index < len(saved_waypoints):
                        current_time = pygame.time.get_ticks()
                        # Check for double-click (within 500ms)
                        if (clicked_index == last_click_index and
                            current_time - last_click_time < 500):
                            # Double-click detected - teleport spectator
                            wp_data = saved_waypoints[clicked_index]
                            wp_loc = wp_data["waypoint"].transform.location
                            teleport_location = carla.Location(
                                x=wp_loc.x,
                                y=wp_loc.y,
                                z=wp_loc.z + 15.0
                            )
                            teleport_rotation = carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)
                            spectator.set_transform(carla.Transform(teleport_location, teleport_rotation))
                            print(f"Teleported to {wp_data['name']}")
                        last_click_time = current_time
                        last_click_index = clicked_index
                    input_active = False
                elif input_box.collidepoint(event.pos):
                    input_active = True
                else:
                    input_active = False

                if button_rect.collidepoint(event.pos) and wp_name.strip():
                    # Save waypoint when button is clicked
                    if export_format == "python":
                        entry = format_waypoint_python(current_waypoint, wp_name) + "\n"
                    else:  # openscenario
                        entry = format_waypoint_openscenario(current_waypoint, wp_name) + "\n"

                    print(entry.strip())

                    # Add to saved waypoints list for visual marking
                    saved_waypoints.append({
                        "name": wp_name,
                        "waypoint": current_waypoint,
                        "index": save_counter
                    })

                    save_counter += 1
                    wp_name = f"Waypoint_{save_counter}"
                    saved_msg_timer = 60  # Show 'Saved' message for 60 frames

                    # Update config
                    config["save_counter"] = save_counter
                    save_config(config)

            # Handle text input
            elif event.type == pygame.KEYDOWN:
                if input_active:
                    if event.key == pygame.K_RETURN:
                        input_active = False
                    elif event.key == pygame.K_BACKSPACE:
                        wp_name = wp_name[:-1]
                    else:
                        wp_name += event.unicode
                else:
                    # Handle keyboard shortcuts when input is not active
                    if event.key == pygame.K_TAB:
                        # Cycle through lane types
                        current_lane_type_index = (current_lane_type_index + 1) % len(lane_types)
                        config["lane_type_index"] = current_lane_type_index
                        save_config(config)
                    elif event.key == pygame.K_s:
                        # Quick save with 'S' key
                        if wp_name.strip():
                            if export_format == "python":
                                entry = format_waypoint_python(current_waypoint, wp_name) + "\n"
                            else:  # openscenario
                                entry = format_waypoint_openscenario(current_waypoint, wp_name) + "\n"
                            print(entry.strip())
                            saved_waypoints.append({
                                "name": wp_name,
                                "waypoint": current_waypoint,
                                "index": save_counter
                            })
                            save_counter += 1
                            wp_name = f"Waypoint_{save_counter}"
                            saved_msg_timer = 60
                            config["save_counter"] = save_counter
                            save_config(config)
                    elif event.key == pygame.K_e:
                        # Toggle export format with 'E' key
                        export_format = "openscenario" if export_format == "python" else "python"
                        config["export_format"] = export_format
                        save_config(config)

        # Display spectator + waypoint info
        draw_text(screen, f"Spectator Location:", (20, 20), font)
        draw_text(
            screen,
            f"X: {location.x:.2f}  Y: {location.y:.2f}  Z: {location.z:.2f}",
            (20, 50),
            font,
        )
        draw_text(
            screen,
            f"Rotation: Pitch: {rotation.pitch:.2f}  Yaw: {rotation.yaw:.2f}  Roll: {rotation.roll:.2f}",
            (20, 80),
            font,
        )

        # Display current lane type and export format
        draw_text(
            screen,
            f"Lane Type: {lane_types[current_lane_type_index][0]} (TAB to cycle)",
            (20, 110),
            font,
            color=(100, 255, 100),
        )
        draw_text(
            screen,
            f"Export Format: {export_format.upper()} (E to toggle)",
            (20, 140),
            font,
            color=(255, 200, 100),
        )

        draw_text(screen, f"Closest Waypoint:", (20, 170), font)
        draw_text(screen, f"Road ID: {current_waypoint.road_id}", (20, 200), font)
        draw_text(screen, f"Lane ID: {current_waypoint.lane_id}", (20, 230), font)
        draw_text(screen, f"Lane Change: {str(current_waypoint.lane_change)}", (20, 260), font)
        draw_text(screen, f"Section ID: {current_waypoint.section_id}", (20, 290), font)
        draw_text(
            screen,
            f"N lanes: {len(get_same_dir_lanes(current_waypoint))}",
            (20, 320),
            font,
        )
        draw_text(
            screen,
            f"N opposite lanes: {len(get_opposite_dir_lanes(current_waypoint))}",
            (20, 350),
            font,
        )
        draw_text(
            screen,
            f"Distance to intersection: {current_waypoint.transform.location.distance(current_waypoint.next_until_lane_end(1)[-1].transform.location):.2f}m",
            (20, 380),
            font,
        )
        draw_text(
            screen,
            f"Saved waypoints: {len(saved_waypoints)}",
            (20, 410),
            font,
            color=(255, 255, 0),
        )

        # Draw input box
        pygame.draw.rect(screen, (255, 255, 255), input_box, 2)
        draw_text(screen, "WP Name:", (input_box.x, input_box.y - 25), font)
        txt_surface = font.render(wp_name, True, (255, 255, 255))
        screen.blit(txt_surface, (input_box.x + 5, input_box.y + 5))

        # Draw save button
        pygame.draw.rect(
            screen,
            button_hover_color if button_rect.collidepoint(mouse_pos) else button_color,
            button_rect,
        )
        draw_text(screen, "Save Waypoint", (button_rect.x + 20, button_rect.y + 15), font)

        # Draw saved message if recent
        if saved_msg_timer > 0:
            draw_text(screen, "Waypoint saved!", (250, 455), font, color=(0, 255, 0))
            saved_msg_timer -= 1

        # Draw waypoint list panel
        pygame.draw.rect(screen, (50, 50, 50), (list_panel_x, list_panel_y, list_panel_width, list_panel_height))
        pygame.draw.rect(screen, (100, 100, 100), (list_panel_x, list_panel_y, list_panel_width, list_panel_height), 2)

        list_font = pygame.font.Font(None, 20)
        draw_text(screen, "Saved Waypoints (double-click to teleport):", (list_panel_x + 5, list_panel_y - 20), list_font, color=(200, 200, 200))

        # Draw waypoint list items
        visible_items = int(list_panel_height / list_item_height)
        for i in range(visible_items):
            wp_index = i + scroll_offset
            if wp_index >= len(saved_waypoints):
                break

            wp_data = saved_waypoints[wp_index]
            item_y = list_panel_y + i * list_item_height

            # Draw item background (alternate colors)
            item_color = (60, 60, 60) if wp_index % 2 == 0 else (70, 70, 70)
            pygame.draw.rect(screen, item_color, (list_panel_x + 2, item_y + 2, list_panel_width - 4, list_item_height - 4))

            # Draw waypoint info
            wp_loc = wp_data["waypoint"].transform.location
            draw_text(screen, f"#{wp_data['index']}: {wp_data['name']}", (list_panel_x + 5, item_y + 5), list_font, color=(255, 255, 100))
            draw_text(screen, f"X:{wp_loc.x:.1f} Y:{wp_loc.y:.1f}", (list_panel_x + 5, item_y + 25), list_font, color=(200, 200, 200))
            draw_text(screen, f"Road:{wp_data['waypoint'].road_id} Lane:{wp_data['waypoint'].lane_id}", (list_panel_x + 5, item_y + 43), list_font, color=(150, 150, 150))

        # Draw help/keyboard shortcuts
        help_font = pygame.font.Font(None, 22)
        draw_text(screen, "Keyboard Shortcuts:", (20, 540), help_font, color=(200, 200, 200))
        draw_text(screen, "TAB: Cycle lane types  |  S: Save waypoint  |  E: Toggle export format", (20, 565), help_font, color=(150, 150, 150))

        # print(f"next_until_lane_end: {current_waypoint.next_until_lane_end(0.1)[-1]} previous_until_lane_start: {current_waypoint.previous_until_lane_start(0.1)[0]}")

        pygame.display.flip()
        clock.tick(30)

    # Save config before exiting
    config["save_counter"] = save_counter
    config["lane_type_index"] = current_lane_type_index
    config["export_format"] = export_format
    save_config(config)

    pygame.quit()


if __name__ == "__main__":
    main()
