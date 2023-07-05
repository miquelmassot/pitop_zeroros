import csv
import time
import math
import sys
from collections import deque
import traceback
import numpy as np
from pitop_zeroros.rplidar import RPLidarImpl as RPLidar
import pygame


def draw_scan(scan_data, screen_radius, surf):
    for angle, dist in scan_data:
        rel_dist = (screen_radius / 2) * (dist / 8000)
        angle_rad = math.radians(angle)

        pos = (
            int(screen_radius / 2) + math.trunc(rel_dist * math.cos(angle_rad)),
            int(screen_radius / 2) + math.trunc(rel_dist * math.sin(angle_rad)),
        )

        pygame.draw.circle(surf, [255, 0, 0], pos, 1)


def exit(lidar):
    lidar.stop_scan()
    lidar.dev.dtr = True
    sys.exit(0)


def calculate_lidar_model(distance):
    return np.array([(distance / 100) * 2, 0, 100, 8000])


def main():
    serial_port = "/dev/ttyUSB0"

    lidar = RPLidar(port=serial_port)

    print("Sent RESET command...")
    lidar.reset()

    time.sleep(1)

    model, fw, hw, serial_no = lidar.get_device_info()
    health_status, err_code = lidar.get_device_health()

    print(
        """
        ===
        Opened LIDAR on serial port {}
        Model ID: {}
        Firmware: {}
        Hardware: {}
        Serial Number: {}
        Device Health Status: {} (Error Code: 0x{:X})
        ===
        """.format(
            serial_port, model, fw, hw, serial_no.hex(), health_status, err_code
        )
    )

    pygame.init()
    clock = pygame.time.Clock()

    font = pygame.font.SysFont("Monospace Regular", 15)

    screen_radius = 500
    screen_size = (screen_radius, screen_radius)
    screen_center = (int(screen_radius / 2), int(screen_radius / 2))
    screen = pygame.display.set_mode(screen_size)

    lidar.start_scan()

    last_scans = deque([], 10)
    cur_scan = []

    surf = pygame.Surface(screen_size)
    surf = surf.convert()

    print("Setting board PWM...")
    lidar.dev.dtr = False

    try:
        while True:
            dt = clock.tick(60)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print("Exiting... (QUIT)")
                    exit(lidar)
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        print("Exiting... (ESCAPE)")
                        exit(lidar)

            polled_samples = lidar.poll_scan_samples()
            if len(polled_samples) > 0:
                print(
                    "[{}] Read {} samples...".format(
                        time.process_time(), len(polled_samples)
                    )
                )

                for angle, dist, quality, new_scan in polled_samples:
                    if new_scan:
                        last_scans.append(cur_scan)
                        cur_scan = []

                    cur_scan.append((angle, dist))

            surf.fill((250,) * 3)

            for angle in range(0, 360, 15):
                angle_rad = math.radians(angle)
                endpt = (
                    int(screen_radius / 2)
                    + math.trunc(screen_radius * math.cos(angle_rad)),
                    int(screen_radius / 2)
                    + math.trunc(screen_radius * math.sin(angle_rad)),
                )

                pygame.draw.line(surf, [128, 128, 128], screen_center, endpt)

                text_angle = math.radians(angle + 1)

                text_pt = (
                    int(screen_radius / 2)
                    + math.trunc(screen_radius * 0.30 * math.cos(text_angle)),
                    int(screen_radius / 2)
                    + math.trunc(screen_radius * 0.30 * math.sin(text_angle)),
                )

                text_surf = font.render(str(angle), False, (0, 0, 0))
                surf.blit(text_surf, text_pt)

            for i in range(1, 9):
                pygame.draw.circle(
                    surf,
                    [128, 128, 128],
                    screen_center,
                    math.trunc((screen_radius / 2) * (i / 8)),
                    2,
                )

                text_pt = (
                    int(screen_radius / 2) + math.trunc((screen_radius / 2) * (i / 8)),
                    int(screen_radius / 2),
                )

                text_surf = font.render(str(i) + "m", False, (0, 0, 0))
                surf.blit(text_surf, text_pt)

            for scan in last_scans:
                draw_scan(scan, screen_radius, surf)

            draw_scan(cur_scan, screen_radius, surf)

            screen.blit(surf, (0, 0))
            pygame.display.flip()
    except KeyboardInterrupt:
        print("Exiting... (KeyboardInterrupt)")
    except Exception as e:
        print("Exiting... (Exception: {})".format(e))
        traceback.print_exc()
    finally:
        exit(lidar)


if __name__ == "__main__":
    main()
