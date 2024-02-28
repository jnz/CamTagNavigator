# pip install pygame
# pip install numpy numpy-stl pygame PyOpenGL
import socket
import numpy as np
import pygame
import time

from render_stl import RenderStlPygame

def parse_message(data):
    """
    Parses the incoming message and returns a dictionary with the parsed data.
    """
    # Split the received message by comma
    parts = data.decode().split(', ')
    if len(parts) != 9:
        print("Invalid message format")
        return None

    # Extract and convert each part of the message
    message = {
        'timestamp_sec': float(parts[0]),
        'udp_message_counter': int(parts[1]),
        'position': {
            'x': float(parts[2]),
            'y': float(parts[3]),
            'z': float(parts[4])
        },
        'quaternion': {
            'w': float(parts[5]),
            'x': float(parts[6]),
            'y': float(parts[7]),
            'z': float(parts[8])
        }
    }

    return message

def start_server(port=5876):
    """
    Starts a UDP server listening on the given port.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', port))
    sock.settimeout(0.005)
    print(f"Listening on port {port}...")

    fps_freelook = False
    pygame.init()
    render = RenderStlPygame("quadcopter.stl")
    render.init("udp3dview")
    clock = pygame.time.Clock()
    # Camera Control
    # Allow FPS style camera movement, capture mouse to do this
    pygame.mouse.set_visible(not fps_freelook)
    pygame.event.set_grab(fps_freelook)

    # Camera variables
    cam_altitude = 0.7
    cam_dist = 5.0 # try to position camera based on rotor plane size
    cam_pos_gl = np.array([-cam_dist, cam_altitude, 0.0]) # camera position in OpenGL system (not! North/East/Down)
    cam_yaw = 90.0 # turn do look down OpenGL x-axis (right) / North
    cam_pitch = 0.0
    MOUSE_SENSITIVITY = 0.1
    MOVE_SPEED = 10.0
    obj_R_b_to_n = np.eye(3) # object attitude  matrix
    obj_pos_n = np.array([0.0,0.0,-cam_altitude]) # object position
    render.render(obj_R_b_to_n, obj_pos_n, cam_pitch, cam_yaw, cam_pos_gl)

    fps_counter = 0
    time_stamp_last_fps_count = time.time()
    last_render_time = time.time()

    while True:
        try:
            data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
            message = parse_message(data)
            if message:
                # print("Received message:", message)
                obj_pos_n[0] = message['position']['z']
                obj_pos_n[1] = message['position']['x']
                obj_pos_n[2] = message['position']['y']
                print("POS %.2f %.2f %.2f (NED)" % (obj_pos_n[0], obj_pos_n[1], obj_pos_n[2]))
        except TimeoutError:
            # no data available
            pass

        current_time = time.time()
        dt_sec = current_time - last_render_time
        last_render_time = current_time

        fps_counter += 1
        if (time.time() - time_stamp_last_fps_count >= 1.0):
            time_stamp_last_fps_count = time.time()
            print("FPS: %i" % (fps_counter))
            fps_counter = 0

        render.render(obj_R_b_to_n, obj_pos_n, cam_pitch, cam_yaw, cam_pos_gl)
        # clock.tick(120)
        if fps_freelook:
            mouse_rel = pygame.mouse.get_rel()
            cam_yaw += mouse_rel[0] * MOUSE_SENSITIVITY
            cam_yaw %= 360.0
            cam_pitch -= mouse_rel[1] * MOUSE_SENSITIVITY
            cam_pitch = max(-89.0, min(89.0, cam_pitch))

        keys = pygame.key.get_pressed()  # Get the state of all keyboard buttons

        # Move forward (along the camera's view direction)
        if keys[pygame.K_s]:
            cam_pos_gl[0] -= dt_sec * MOVE_SPEED * np.sin(np.deg2rad(cam_yaw))
            cam_pos_gl[2] += dt_sec * MOVE_SPEED * np.cos(np.deg2rad(cam_yaw))
        # Move backward
        if keys[pygame.K_w]:
            cam_pos_gl[0] += dt_sec * MOVE_SPEED * np.sin(np.deg2rad(cam_yaw))
            cam_pos_gl[2] -= dt_sec * MOVE_SPEED * np.cos(np.deg2rad(cam_yaw))
        # Move left
        if keys[pygame.K_a]:
            cam_pos_gl[0] += dt_sec * MOVE_SPEED * np.sin(np.deg2rad(cam_yaw - 90.0))
            cam_pos_gl[2] -= dt_sec * MOVE_SPEED * np.cos(np.deg2rad(cam_yaw - 90.0))
        # Move right
        if keys[pygame.K_d]:
            cam_pos_gl[0] += dt_sec * MOVE_SPEED * np.sin(np.deg2rad(cam_yaw + 90.0))
            cam_pos_gl[2] -= dt_sec * MOVE_SPEED * np.cos(np.deg2rad(cam_yaw + 90.0))
        if keys[pygame.K_q]:
            cam_pos_gl[1] += dt_sec * MOVE_SPEED
        if keys[pygame.K_e]:
            cam_pos_gl[1] -= dt_sec * MOVE_SPEED

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                g_sim_running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    g_sim_running = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                fps_freelook = not fps_freelook
                pygame.mouse.set_visible(not fps_freelook)
                pygame.event.set_grab(fps_freelook)
                mouse_rel = pygame.mouse.get_rel() # consume one get_rel to avoid jump
                if fps_freelook:
                    print("Freelook active, press mousebutton to exit mode")
                else:
                    print("Freelook disabled")


if __name__ == "__main__":
    start_server()

