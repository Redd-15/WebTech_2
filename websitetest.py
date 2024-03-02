from flask import Flask, render_template, Response, request
import cv2, numpy as np
import serial, time, datetime, copy
from threading import Thread

IP = '192.168.0.108'

ser = serial.Serial('/dev/ttyS4', 115200, timeout = 5)

ONE_FRAME_MOVE = [25,25,100]
CAMID = 0
SHAPE_OF_FRAME = [480, 640]
SHAPE_OF_CUT = [100,100]
OFFSET_OF_FRAME_LT = [int((SHAPE_OF_FRAME[0]-SHAPE_OF_CUT[0])/2), int((SHAPE_OF_FRAME[1]-SHAPE_OF_CUT[1])/2)]
OFFSET_OF_FRAME_RB = [int(SHAPE_OF_FRAME[0]-((SHAPE_OF_FRAME[0]-SHAPE_OF_CUT[0])/2)), int(SHAPE_OF_FRAME[1]-((SHAPE_OF_FRAME[1]-SHAPE_OF_CUT[1])/2))]

WEBSITE_PAGE = 0

STOP = False
FILE_NAME = str(datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))
CALIB_CANVAS = [40, 40]
LAST_ERR = ""

focus_plane = None
focus_point_coords = [[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
focus_phase = 0
enable_autofocus = False
FOCUS_CALIBRATED = False
end_of_canvas_line = False

v_cal_done = True
h_cal_done = True
calib_done = True
stp = 1
step_frame = True
coords = [0, 0, 0]

enable_pol_cam = False
camera_type = 0
Started_image_rendering = False


vid = cv2.VideoCapture(CAMID)


while(not vid.isOpened()):
    CAMID = CAMID + 1
    print(f'Attempting to connect to camera at port no. {CAMID}')
    vid = cv2.VideoCapture(CAMID)

    if (CAMID > 9):
        print(f'Camera is Disconnected!') 
        exit(-1)       

print(f'Connected to camera at port no. {CAMID}')


x = None
ser.write(str.encode(f'xe0001'))
x = ser.read(4)
ser.write(str.encode(f'xh0001'))
x = ser.read(4)

if x == b'':
    print("Motor controller is Disconnected!")
    exit(-2)
else:
    print(f'Motor controller connected!')


def cmd_conv(move):

    str_move = str(move)
    offset = 4 - len(str_move)
    chars = ['0','0','0','0']

    for i in range(len(str_move)):
        chars[i+offset] = str_move[i]
    output = ""
    for i in range(len(chars)):
        output += chars[i]

    return output

def inerpInt1D(x, in_min, in_max, out_min, out_max):
  return round((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def focus():

    global frame_acq, STOP, LAST_ERR, camera_type
    delta = 0
    LIFE_EXPECTANCY = 3
    life = LIFE_EXPECTANCY
    
    camera_type = 2
    time.sleep(0.8)
    focus_null = calculate_focus_score(frame_edges[OFFSET_OF_FRAME_LT[0]:OFFSET_OF_FRAME_RB[0], OFFSET_OF_FRAME_LT[1]:OFFSET_OF_FRAME_RB[1]])

    manual_move("F+")
    time.sleep(0.8)
    delta = delta + 1
    focus_up = calculate_focus_score(frame_edges[OFFSET_OF_FRAME_LT[0]:OFFSET_OF_FRAME_RB[0], OFFSET_OF_FRAME_LT[1]:OFFSET_OF_FRAME_RB[1]])
    
    manual_move("F-", 2)
    time.sleep(0.8)
    delta = delta - 2

    focus_down = calculate_focus_score(frame_edges[OFFSET_OF_FRAME_LT[0]:OFFSET_OF_FRAME_RB[0], OFFSET_OF_FRAME_LT[1]:OFFSET_OF_FRAME_RB[1]])

    focus_sc_max = max(focus_null, focus_up, focus_down)

    if focus_sc_max == focus_null:
        if focus_sc_max == 0:
            LAST_ERR = "MANUAL FOCUS REQUIRED"
        manual_move("F+")
        camera_type = 0
        return 0
    
    elif focus_sc_max == focus_down:
        dir = "F-"
        manual_move("F-")
        time.sleep(1)
    
    else:
        dir = "F+"
        manual_move("F+", 3)
        time.sleep(1.2)

    focus_sc = calculate_focus_score(frame_edges[OFFSET_OF_FRAME_LT[0]:OFFSET_OF_FRAME_RB[0], OFFSET_OF_FRAME_LT[1]:OFFSET_OF_FRAME_RB[1]])

    while life > 0 and not STOP:
        manual_move(dir, 1)
        if dir == "F+":
            delta = delta + 1
        elif dir == "F-":
            delta = delta - 1
        
        time.sleep(0.8)
        focus_sc = calculate_focus_score(frame_edges[OFFSET_OF_FRAME_LT[0]:OFFSET_OF_FRAME_RB[0], OFFSET_OF_FRAME_LT[1]:OFFSET_OF_FRAME_RB[1]])

        if focus_sc < focus_sc_max:
            life = life - 1
        else:
            focus_sc_max = focus_sc
            life = LIFE_EXPECTANCY

    if dir == "F+":
        manual_move("F-", LIFE_EXPECTANCY)
    elif dir == "F-":
        manual_move("F+", LIFE_EXPECTANCY)

    camera_type = 0
    if not STOP:
        return delta
    else:
        return 0

def is_even(num):
    if (num % 2) == 0:
        return True
    else:
        return False

def change_shape_of_cut(x):
    global SHAPE_OF_CUT, OFFSET_OF_FRAME_LT, OFFSET_OF_FRAME_RB
    
    SHAPE_OF_CUT[0], SHAPE_OF_CUT[1] = (x,x)
    OFFSET_OF_FRAME_LT = [int((SHAPE_OF_FRAME[0]-SHAPE_OF_CUT[0])/2), int((SHAPE_OF_FRAME[1]-SHAPE_OF_CUT[1])/2)]
    OFFSET_OF_FRAME_RB = [int(SHAPE_OF_FRAME[0]-((SHAPE_OF_FRAME[0]-SHAPE_OF_CUT[0])/2)), int(SHAPE_OF_FRAME[1]-((SHAPE_OF_FRAME[1]-SHAPE_OF_CUT[1])/2))]

def focus_by_plane():

    global coords, focus_plane

    step = coords[2] - focus_plane[coords[0]][coords[1]]
    if step < 0 :
        manual_move("F+", abs(step))

    elif step > 0 :
        manual_move("F-", abs(step))

    coords[2] = coords[2] - step

def calc_focus_points():        ##################
    global focus_point_coords, coords

    focus_point_coords[1] = [focus_point_coords[3][0], focus_point_coords[0][1], 0]
    focus_point_coords[2] = [focus_point_coords[0][0], focus_point_coords[3][1], 0]
    
    move_to_coord([focus_point_coords[1][0],focus_point_coords[1][1]])
    time.sleep(1)
    focus_point_coords[1][2] = focus_point_coords[3][2] + focus()

    move_to_coord([focus_point_coords[0][0],focus_point_coords[0][1]])
    time.sleep(1)
    focus_point_coords[0][2] = focus_point_coords[1][2] + focus()

    move_to_coord([focus_point_coords[2][0],focus_point_coords[2][1]])
    time.sleep(1)
    focus_point_coords[2][2] = focus_point_coords[0][2] + focus()

    coords[2] = focus_point_coords[2][2]

    move_to_coord([0,0])


def calc_foc_plane(debug = False):

    global focus_point_coords, CALIB_CANVAS, focus_plane
    
    focus_plane = np.zeros([CALIB_CANVAS[0], CALIB_CANVAS[1]], dtype=np.int8)
    
    for x in range(focus_point_coords[0][0]+1):
        for y in range(focus_point_coords[0][1]+1):
            focus_plane[x][y] = focus_point_coords[0][2]
    
    for x in range(focus_point_coords[1][0],CALIB_CANVAS[0]):
        for y in range(focus_point_coords[1][1]+1):
            focus_plane[x][y] = focus_point_coords[1][2]
    
    for x in range(focus_point_coords[2][0]+1):
        for y in range(focus_point_coords[2][1], CALIB_CANVAS[1]):
            focus_plane[x][y] = focus_point_coords[2][2]
    
    for x in range(focus_point_coords[3][0], CALIB_CANVAS[0]):
        for y in range(focus_point_coords[3][1], CALIB_CANVAS[1]):
            focus_plane[x][y] = focus_point_coords[3][2]

    for i in range(focus_point_coords[0][0],focus_point_coords[1][0]):
        tobeset = inerpInt1D(i, focus_point_coords[0][0],focus_point_coords[1][0], focus_point_coords[0][2],focus_point_coords[1][2]) 
        focus_plane[i][0:focus_point_coords[0][1]+1] = tobeset

    for i in range(focus_point_coords[2][0],focus_point_coords[3][0]):
        tobeset = inerpInt1D(i, focus_point_coords[2][0],focus_point_coords[3][0], focus_point_coords[2][2],focus_point_coords[3][2]) 
        focus_plane[i][focus_point_coords[2][1]:CALIB_CANVAS[1]] = tobeset

    for i in range(focus_point_coords[0][1],focus_point_coords[2][1]):
        tobeset = inerpInt1D(i, focus_point_coords[0][1],focus_point_coords[2][1], focus_point_coords[0][2],focus_point_coords[2][2]) 
        for tick in range(0, focus_point_coords[0][0]+1):
            focus_plane[tick][i] = tobeset

    for i in range(focus_point_coords[1][1],focus_point_coords[3][1]):
        tobeset = inerpInt1D(i, focus_point_coords[1][1],focus_point_coords[3][1], focus_point_coords[1][2],focus_point_coords[3][2]) 
        for tick in range(focus_point_coords[1][0], CALIB_CANVAS[0]):
            focus_plane[tick][i] = tobeset

    for ix in range(focus_point_coords[0][0]+1,focus_point_coords[1][0]):
        for iy in range(focus_point_coords[1][1],focus_point_coords[3][1]):
            tobeset = inerpInt1D(iy, focus_point_coords[1][1],focus_point_coords[3][1], focus_plane[ix][focus_point_coords[1][1]],focus_plane[ix][focus_point_coords[3][1]]) 
            focus_plane[ix][iy] = tobeset

    if debug:
        for y in range(CALIB_CANVAS[1]):
            for x in range(CALIB_CANVAS[0]):
                print(str(round(focus_plane[x][y])) + '|', end='')
            print()


def polarize(dir, debug = False):
    global ser

    if dir == "on":
        ser.write(str.encode(f'pk0000'))    
    elif dir == "off":
        ser.write(str.encode(f'pb0000'))    
    
    x = ser.read(4)
    time.sleep(0.1)

    if debug:
        print(f'dir: {dir} output: {x}')

def move_to_coord(point):

    global coords
    delta = [point[0]-coords[0],point[1]-coords[1]]

    manual_move("x-" if delta[0] > 0 else "x+", abs(delta[0]))
    manual_move("y-" if delta[1] > 0 else "y+", abs(delta[1]))

def motor_control():

    global Started_image_rendering, enable_autofocus, FILE_NAME
    global coords, STOP, WEBSITE_PAGE

    print("Started Mot_Ctrl thread")

    while True:
        
        going_left = False
        app_num = 1 #add_pic_part number!

        while Started_image_rendering:

            if (coords[0] == 0 and going_left) or coords[0] == (CALIB_CANVAS[0]-1):
                if coords[1] == CALIB_CANVAS[1]-1:
                    
                    add_pic_part(app_num)
                    time.sleep(0.4)
                    
                    move_to_coord([0,0])

                    while(coords[0] != 0 and coords[1] != 0):
                        time.sleep(0.1)

                    going_left = False

                    if not enable_pol_cam or (enable_pol_cam and app_num == 2):
                        Started_image_rendering = False
                        FILE_NAME = str(datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))
                        break

                    if enable_pol_cam and app_num == 1:
                        time.sleep(0.5)
                        polarize("on")
                        time.sleep(0.5)
                        app_num = 2

                else:
                    
                    add_pic_part(app_num)

                    manual_move("y-")

                    if enable_autofocus:
                        focus_by_plane()

                    time.sleep(0.4)
                    going_left = not going_left                    

            if going_left:
                add_pic_part(app_num, True)

                manual_move("x+")

                if enable_autofocus:
                    focus_by_plane()

                time.sleep(0.4)
                
            else: 
                add_pic_part(app_num, False)

                manual_move("x-")

                if enable_autofocus:
                    focus_by_plane()

                time.sleep(0.4)


def camera_frame_acq():
    
    global vid, frame_acq, frame_acq_gray, v_cal_done, h_cal_done, LAST_ERR
    global camera_type, frame_edges, lines_on_frame, num_vert_lines_on_frame, num_hor_lines_on_frame

    print("Started Frame_acq thread")

    while True:

        if not vid.isOpened():
            print(f'Reconnecting to camera at ID: {CAMID}')
            vid = cv2.VideoCapture(CAMID)

        ret, frame_acq = vid.read() 
        frame_acq_gray = cv2.cvtColor(frame_acq, cv2.COLOR_BGR2GRAY)

        if not ret:
            LAST_ERR = "Camera has disconnected!"
            vid.release()
            break
        
        lines_on_frame = copy.deepcopy(frame_acq)

        if camera_type != 0 or v_cal_done == False or h_cal_done == False:
            frame_edges = cv2.Canny(frame_acq_gray, 130, 130, apertureSize = 3)

            lines = cv2.HoughLinesP(image=frame_edges,rho=1,theta=np.pi/180, threshold=100,lines=np.array([]), minLineLength=100,maxLineGap=100)

            try:
                num_vert_lines_on_frame = 0
                num_hor_lines_on_frame = 0

                for i in range(min(lines.shape[0],5)):
                    x1,y1,x2,y2 = lines[i][0]
                    x_diff = (x2 - x1)
                    y_diff = (y2 - y1)
                    if (abs(x_diff) < 100) and ((x1 > OFFSET_OF_FRAME_LT[1] and x1 < OFFSET_OF_FRAME_RB[1]) or (x2 > OFFSET_OF_FRAME_LT[1] and x2 < OFFSET_OF_FRAME_RB[1])):
                        num_vert_lines_on_frame = num_vert_lines_on_frame + 1
                        cv2.line(lines_on_frame, (x1, y1), (x2, y2), (0, 0, 255), 3, cv2.LINE_AA)

                    elif (abs(y_diff) < 100) and ((y1 > OFFSET_OF_FRAME_LT[0] and y1 < OFFSET_OF_FRAME_RB[0]) or (y2 > OFFSET_OF_FRAME_LT[0] and y2 < OFFSET_OF_FRAME_RB[0])):
                        num_hor_lines_on_frame = num_hor_lines_on_frame + 1
                        cv2.line(lines_on_frame, (x1, y1), (x2, y2), (255, 0, 0), 3, cv2.LINE_AA)

            except:
                num_vert_lines_on_frame = 0
                num_hor_lines_on_frame = 0

def overlay_frame_manual():
    global frame_acq, frame_overlay, SHAPE_OF_CUT, SHAPE_OF_FRAME, OFFSET_OF_FRAME_LT, OFFSET_OF_FRAME_RB

    prev_frame = np.zeros([SHAPE_OF_CUT[0]+SHAPE_OF_FRAME[0], SHAPE_OF_CUT[1]+SHAPE_OF_FRAME[1], 3], dtype = np.uint8)    
    prev_frame[0:SHAPE_OF_FRAME[0], 0:SHAPE_OF_FRAME[1], :3] = frame_acq

    time.sleep(1)
    manual_move("x-")
    manual_move("y-")
    time.sleep(1)

    new_frame = np.zeros([SHAPE_OF_CUT[0]+SHAPE_OF_FRAME[0], SHAPE_OF_CUT[1]+SHAPE_OF_FRAME[1], 3], dtype = np.uint8)
    new_frame[SHAPE_OF_CUT[0]:SHAPE_OF_CUT[0]+SHAPE_OF_FRAME[0], SHAPE_OF_CUT[1]:SHAPE_OF_CUT[1]+SHAPE_OF_FRAME[1], :3] = frame_acq

    time.sleep(1)
    manual_move("x+")
    manual_move("y+")

    frame_overlay = cv2.addWeighted(prev_frame, 0.5, new_frame, 0.5, 0.0)


def overlay_frame_automatic():
    global frame_acq, frame_overlay
    global STOP, SHAPE_OF_CUT, SHAPE_OF_FRAME, OFFSET_OF_FRAME_LT, OFFSET_OF_FRAME_RB
    dir = True
    LIFE_EXPECTANCY = 2
    life = LIFE_EXPECTANCY


    def get_score():
    
        global frame_overlay

        prev_frame = np.zeros([SHAPE_OF_CUT[0]+SHAPE_OF_FRAME[0], SHAPE_OF_CUT[1]+SHAPE_OF_FRAME[1], 3], dtype = np.uint8)    
        prev_frame[0:SHAPE_OF_FRAME[0], 0:SHAPE_OF_FRAME[1], :3] = frame_acq

        time.sleep(1)
        manual_move("x-")
        manual_move("y-")
        time.sleep(1)

        new_frame = np.zeros([SHAPE_OF_CUT[0]+SHAPE_OF_FRAME[0], SHAPE_OF_CUT[1]+SHAPE_OF_FRAME[1], 3], dtype = np.uint8)
        new_frame[SHAPE_OF_CUT[0]:SHAPE_OF_CUT[0]+SHAPE_OF_FRAME[0], SHAPE_OF_CUT[1]:SHAPE_OF_CUT[1]+SHAPE_OF_FRAME[1], :3] = frame_acq

        time.sleep(1)
        manual_move("x+")
        manual_move("y+")
        time.sleep(1)

        frame_overlay = cv2.addWeighted(prev_frame, 0.5, new_frame, 0.5, 0.0)

        return calculate_blur_score(frame_overlay[SHAPE_OF_CUT[0]:SHAPE_OF_FRAME[0], SHAPE_OF_CUT[1]:SHAPE_OF_FRAME[1], :3])
    
    prev_score = get_score()
    change_shape_of_cut(SHAPE_OF_CUT[0] + 2)
    score = get_score()

    if score < prev_score:
        dir = False
        change_shape_of_cut(SHAPE_OF_CUT[0] - 4)
        score = get_score()


    while life > 0 and not STOP:
        
        if dir:
            change_shape_of_cut(SHAPE_OF_CUT[0] + 2)
        else: 
            change_shape_of_cut(SHAPE_OF_CUT[0] - 2)
            
        prev_score = score
        score = get_score()

        if score < prev_score:
            life = life - 1
        else:
            life = LIFE_EXPECTANCY

    if dir:
        change_shape_of_cut(SHAPE_OF_CUT[0] - (2 * LIFE_EXPECTANCY))
    else: 
        change_shape_of_cut(SHAPE_OF_CUT[0] + (2 * LIFE_EXPECTANCY))

    if not STOP:
        get_score()


def vertical_calibration():

    global num_vert_lines_on_frame, CALIB_CANVAS, STOP
    global v_cal_done, ONE_FRAME_MOVE, LAST_ERR, end_of_canvas_line

    v_cal_done = False
    time.sleep(0.5)
    width = 0


    if num_vert_lines_on_frame != 0:  

        while not v_cal_done and not STOP:


            if width > 100:
                raise Exception()
            
            manual_move("x-")
            width = width + 1
            time.sleep(0.4)
            if num_vert_lines_on_frame != 0 and width > 5 or end_of_canvas_line:
                v_cal_done = True

                CALIB_CANVAS[0] = width
                end_of_canvas_line = False

                manual_move("x+", width)

                time.sleep(1)
    else: 
        LAST_ERR = "No vertical edge detected!"
        v_cal_done = True
        raise Exception()

def horizontal_calibration():

    global num_hor_lines_on_frame, CALIB_CANVAS, STOP
    global h_cal_done, ONE_FRAME_MOVE, LAST_ERR, end_of_canvas_line

    h_cal_done = False
    time.sleep(0.5)
    height = 0

    if num_hor_lines_on_frame != 0:  

        while not h_cal_done and not STOP:
            
            if height > 100:
                raise Exception()
            manual_move("y-")
            height = height + 1
            time.sleep(0.4)
            if num_hor_lines_on_frame != 0 and height > 5 or end_of_canvas_line:
                h_cal_done = True

                CALIB_CANVAS[1] = height
                end_of_canvas_line = False

                manual_move("y+", height)
                
                time.sleep(1)
    else: 
        LAST_ERR = "No horizontal edge detected!"
        h_cal_done = True
        raise Exception()

def full_calibration_mid(mid):

    global num_hor_lines_on_frame, STOP, h_cal_done, v_cal_done, end_of_canvas_line

    if mid:
        v_cal_done = False
        manual_move("x-", 5)
        time.sleep(0.8)
        
        while not STOP and num_hor_lines_on_frame == 0 and not end_of_canvas_line:
            manual_move("y+")
            time.sleep(0.8)
        
        v_cal_done = True
    
    else:
        h_cal_done = False
        manual_move("x+", 5)
        h_cal_done = True

        time.sleep(0.8)

def manual_move(dir, amount = 1, step = True, debug = False):

    global ser, STOP, LAST_ERR, WEBSITE_PAGE, coords, ONE_FRAME_MOVE

    if debug:
        print(f'Direction: {dir} | Amount: {amount} | Step whole frame: {step}')

    if step == True:
        movement = ONE_FRAME_MOVE
    else:
        movement = (amount, amount, amount)
    
    if dir == "←":
        dir = "x+"
    elif dir == "→":
        dir = "x-"
    elif dir == "↑":
        dir = "y+"
    elif dir == "↓":
        dir = "y-"

    while amount > 0 and not STOP:
        time.sleep(0.2)
        x = None
        match dir:
            case "x+":
                ser.write(str.encode(f'xe{cmd_conv(movement[0])}'))
                x = ser.read(4)
                if WEBSITE_PAGE != 0:
                    coords[0] = coords[0] - 1
                    
            case "x-":
                ser.write(str.encode(f'xh{cmd_conv(movement[0])}'))
                x = ser.read(4)
                if WEBSITE_PAGE != 0:
                    coords[0] = coords[0] + 1

            case "y+":
                ser.write(str.encode(f'ye{cmd_conv(movement[1])}'))
                x = ser.read(4)
                if WEBSITE_PAGE != 0:
                    coords[1] = coords[1] - 1

            case "y-":
                ser.write(str.encode(f'yh{cmd_conv(movement[1])}'))
                x = ser.read(4)
                if WEBSITE_PAGE != 0:
                    coords[1] = coords[1] + 1

            case "F+":
                ser.write(str.encode(f'ze{cmd_conv(movement[2])}'))
                x = ser.read(4)
            case "F-":
                ser.write(str.encode(f'zh{cmd_conv(movement[2])}'))
                x = ser.read(4)
        
        if x == b'':
            LAST_ERR = "Motor controller error!"
            exit(-2)

        amount = amount - 1
        
        if not step:
            amount = 0
        
        if debug :
            print(x)

def save_image(frame, type):
    global enable_pol_cam
    if type == 1:
        if enable_pol_cam:
            cv2.imwrite(str(full_file_name("/home/microscope/smbshare/Rendered/" + FILE_NAME, "_sim")), frame)
        else:
            cv2.imwrite(str(full_file_name("/home/microscope/smbshare/Rendered/" + FILE_NAME)), frame)

    if type == 2:
        cv2.imwrite(str(full_file_name("/home/microscope/smbshare/Rendered/" + FILE_NAME, "_pol")), frame)

def calculate_blur_score(image):
   image_filtered = cv2.medianBlur(image, 9)
   laplacian = cv2.Laplacian(image_filtered, cv2.CV_64F)
   blur_score = laplacian.var()
   return blur_score

def calculate_focus_score(image):
   focus_score = np.sum(image == 255)
   return focus_score

def edge_smoothing(frame_new, going_left):

    global coords, SHAPE_OF_CUT, CALIB_CANVAS

    if coords[1] !=0:
        frame_corr_1 = frame_new[(coords[1]*SHAPE_OF_CUT[1]),((coords[0])*SHAPE_OF_CUT[0]):((coords[0]+1)*SHAPE_OF_CUT[0]), :3]
        frame_corr_2 = frame_new[(coords[1]*SHAPE_OF_CUT[1])-1,((coords[0])*SHAPE_OF_CUT[0]):((coords[0]+1)*SHAPE_OF_CUT[0]), :3]
        corrected_line = cv2.addWeighted(frame_corr_1, 0.5,frame_corr_2, 0.5, 0.0)

        frame_new[(coords[1]*SHAPE_OF_CUT[1]),((coords[0])*SHAPE_OF_CUT[0]):((coords[0]+1)*SHAPE_OF_CUT[0]), :3] = corrected_line
        frame_new[(coords[1]*SHAPE_OF_CUT[1])-1,((coords[0])*SHAPE_OF_CUT[0]):((coords[0]+1)*SHAPE_OF_CUT[0]), :3] = corrected_line

    if going_left is not None:

        if going_left and coords[0] != CALIB_CANVAS[0]-1:
            frame_corr_1 = frame_new[(coords[1]*SHAPE_OF_CUT[1]):((coords[1]+1)*SHAPE_OF_CUT[1]),((coords[0]+1)*SHAPE_OF_CUT[0]), :3]
            frame_corr_2 = frame_new[(coords[1]*SHAPE_OF_CUT[1]):((coords[1]+1)*SHAPE_OF_CUT[1]),((coords[0]+1)*SHAPE_OF_CUT[0])-1, :3]
            corrected_line = cv2.addWeighted(frame_corr_1, 0.5,frame_corr_2, 0.5, 0.0)

            frame_new[(coords[1]*SHAPE_OF_CUT[1]):((coords[1]+1)*SHAPE_OF_CUT[1]),((coords[0]+1)*SHAPE_OF_CUT[0]), :3] = corrected_line
            frame_new[(coords[1]*SHAPE_OF_CUT[1]):((coords[1]+1)*SHAPE_OF_CUT[1]),((coords[0]+1)*SHAPE_OF_CUT[0])-1, :3] = corrected_line
            
        elif  not going_left and coords[0] != 0:
            frame_corr_1 = frame_new[(coords[1]*SHAPE_OF_CUT[1]):((coords[1]+1)*SHAPE_OF_CUT[1]),(coords[0]*SHAPE_OF_CUT[0]), :3]
            frame_corr_2 = frame_new[(coords[1]*SHAPE_OF_CUT[1]):((coords[1]+1)*SHAPE_OF_CUT[1]),(coords[0]*SHAPE_OF_CUT[0])-1, :3]
            corrected_line = cv2.addWeighted(frame_corr_1, 0.5,frame_corr_2, 0.5, 0.0)

            frame_new[(coords[1]*SHAPE_OF_CUT[1]):((coords[1]+1)*SHAPE_OF_CUT[1]),(coords[0]*SHAPE_OF_CUT[0]), :3] = corrected_line
            frame_new[(coords[1]*SHAPE_OF_CUT[1]):((coords[1]+1)*SHAPE_OF_CUT[1]),(coords[0]*SHAPE_OF_CUT[0])-1, :3] = corrected_line
        
    return frame_new

def add_pic_part(type, going_left = None):
    
    global Started_image_rendering, frame_acq, frame_sim, frame_pol
    global coords, CALIB_CANVAS
    
    if Started_image_rendering:
        
        if type == 1:
            if frame_sim is None:
                frame_sim = frame_acq[OFFSET_OF_FRAME_LT[0]:OFFSET_OF_FRAME_RB[0], OFFSET_OF_FRAME_LT[1]:OFFSET_OF_FRAME_RB[1], :3]

            h1, w1 = frame_sim.shape[:2]
                
            frame_new = np.zeros(((CALIB_CANVAS[1])*SHAPE_OF_CUT[1], (CALIB_CANVAS[0])*SHAPE_OF_CUT[0], 3), np.uint8)
            
            frame_new[:h1, :w1, :3] = frame_sim

            frame_new[(coords[1]*SHAPE_OF_CUT[1]):((coords[1]+1)*SHAPE_OF_CUT[1]), (coords[0]*SHAPE_OF_CUT[0]):((coords[0]+1)*SHAPE_OF_CUT[0]), :3] = frame_acq[OFFSET_OF_FRAME_LT[0]:OFFSET_OF_FRAME_RB[0], OFFSET_OF_FRAME_LT[1]:OFFSET_OF_FRAME_RB[1], :3]

            frame_new = edge_smoothing(frame_new, going_left)
                
            frame_sim = frame_new

            save_image(frame_sim, 1)
            
        if type == 2:
            if frame_pol is None:
                frame_pol = frame_acq[OFFSET_OF_FRAME_LT[0]:OFFSET_OF_FRAME_RB[0], OFFSET_OF_FRAME_LT[1]:OFFSET_OF_FRAME_RB[1], :3]

            h1, w1 = frame_pol.shape[:2]
                
            frame_new_pol = np.zeros(((CALIB_CANVAS[1])*SHAPE_OF_CUT[1], (CALIB_CANVAS[0])*SHAPE_OF_CUT[0], 3), np.uint8)
            
            frame_new_pol[:h1, :w1, :3] = frame_pol

            frame_new_pol[(coords[1]*SHAPE_OF_CUT[1]):((coords[1]+1)*SHAPE_OF_CUT[1]), (coords[0]*SHAPE_OF_CUT[0]):((coords[0]+1)*SHAPE_OF_CUT[0]), :3] = frame_acq[OFFSET_OF_FRAME_LT[0]:OFFSET_OF_FRAME_RB[0], OFFSET_OF_FRAME_LT[1]:OFFSET_OF_FRAME_RB[1], :3]
            
            frame_new_pol = edge_smoothing(frame_new_pol, going_left)

            frame_pol = frame_new_pol

            save_image(frame_pol, 2)

def full_file_name(name, plus = ""):
        return "" + str(name) + str(plus) + ".jpg"


app = Flask(__name__)###################################################################################################################################################################################################################################################

def get_response_page():

    global enable_autofocus, enable_pol_cam, CALIB_CANVAS, FILE_NAME, LAST_ERR, focus_point_coords
    global stp, step_frame, WEBSITE_PAGE, FOCUS_CALIBRATED, focus_phase, SHAPE_OF_CUT

    match WEBSITE_PAGE:
        case 0:

            return render_template('main_page.html',
                checked_af = "checked" if enable_autofocus else " ",     
                checked_pol = "checked" if enable_pol_cam else " ",
                x_data = CALIB_CANVAS[0],
                y_data = CALIB_CANVAS[1],
                file_name = FILE_NAME,
                stp_data = stp,
                ERROR = LAST_ERR,
                checked_step = "checked" if step_frame else " ",
                psize = SHAPE_OF_CUT[0])
        case 1:

            return render_template('render_page.html',
                x_data = CALIB_CANVAS[0],
                y_data = CALIB_CANVAS[1],
                file_name = FILE_NAME,
                ERROR = LAST_ERR)
        case 2:

            
            return render_template('focus_cal_page.html',
                stp_data = stp,
                x_data = CALIB_CANVAS[0],
                y_data = CALIB_CANVAS[1],
                file_name = FILE_NAME,
                ERROR = LAST_ERR,
                focus_coords= "P1: "+str(focus_point_coords[0][0])+";"+str(focus_point_coords[0][1])+" | P2: "+str(focus_point_coords[3][0])+";"+str(focus_point_coords[3][1]),
                focus_step="Set P1" if focus_phase == 0 else "Set P2")


def get_frame_live():  # generate frame by frame from camera
    
    global camera_type, frame_acq, frame_edges, frame_overlay, lines_on_frame
    global OFFSET_OF_FRAME_LT, OFFSET_OF_FRAME_RB
    frame_overlay = None

    while True:

        if camera_type != 3:
            match camera_type:
                case 0:
                    frame = copy.deepcopy(frame_acq)
                case 1:
                    frame = copy.deepcopy(lines_on_frame)
                case 2:
                    try:
                        frame = copy.deepcopy(frame_edges)
                    except:
                        frame = copy.deepcopy(frame_acq)

            ret, buffer = cv2.imencode('.png', cv2.rectangle(frame, (OFFSET_OF_FRAME_LT[1], OFFSET_OF_FRAME_LT[0]), (OFFSET_OF_FRAME_RB[1],OFFSET_OF_FRAME_RB[0]), (0,0,255), 1))
            frame_OUT = buffer.tobytes()

        else:
            if frame_overlay is not None:
                ret, buffer = cv2.imencode('.png', frame_overlay)
            else:
                ret, buffer = cv2.imencode('.png', frame_acq)
            
            frame_OUT = buffer.tobytes()

        yield (b'--frame\r\n'
            b'Content-Type: image/png\r\n\r\n' + frame_OUT + b'\r\n') 
    
def get_frame_sim():

    global Started_image_rendering, frame_sim
    
    stop_pic = cv2.imread("/home/microscope/smbshare/MicroscopeGUI/templates/feature_img_sample.jpg", 1)
    pic = stop_pic
    pic_prev = stop_pic
    
    while True:
        
        try:
            if not Started_image_rendering:
                pic = stop_pic
                ret, buffer = cv2.imencode('.png', pic)
                frame_OUT = buffer.tobytes()
                pic_prev = pic
                yield (b'--frame\r\n'
                        b'Content-Type: image/png\r\n\r\n' + frame_OUT + b'\r\n') 
            
            else:
                ret, buffer = cv2.imencode('.png', frame_sim)
                frame_OUT = buffer.tobytes()
                pic_prev = pic
                yield (b'--frame\r\n'
                        b'Content-Type: image/png\r\n\r\n' + frame_OUT + b'\r\n') 
            
        except Exception as e:
            ret, buffer = cv2.imencode('.png', pic_prev)
            frame_OUT = buffer.tobytes()
            yield (b'--frame\r\n'
                    b'Content-Type: image/png\r\n\r\n' + frame_OUT + b'\r\n')
            
def get_frame_pol():

    global Started_image_rendering, frame_pol
    
    stop_pic = cv2.imread("/home/microscope/smbshare/MicroscopeGUI/templates/feature_img_sample.jpg", 1)
    pic = stop_pic
    pic_prev = stop_pic
    
    while True:
        
        try:
            if not Started_image_rendering:
                pic = stop_pic
                ret, buffer = cv2.imencode('.png', pic)
                frame_OUT = buffer.tobytes()
                pic_prev = pic
                yield (b'--frame\r\n'
                        b'Content-Type: image/png\r\n\r\n' + frame_OUT + b'\r\n') 
            
            else:
                ret, buffer = cv2.imencode('.png', frame_pol)
                frame_OUT = buffer.tobytes()
                pic_prev = pic
                yield (b'--frame\r\n'
                        b'Content-Type: image/png\r\n\r\n' + frame_OUT + b'\r\n') 
            
        except Exception as e:
            ret, buffer = cv2.imencode('.png', pic_prev)
            frame_OUT = buffer.tobytes()
            yield (b'--frame\r\n'
                    b'Content-Type: image/png\r\n\r\n' + frame_OUT + b'\r\n')


@app.route("/", methods=['GET', 'POST'])

@app.route("/home", methods=['GET', 'POST'])
def about():
    
    global Started_image_rendering, frame_pol, frame_sim, coords, camera_type
    global enable_autofocus, enable_pol_cam, CALIB_CANVAS, STOP, FILE_NAME, LAST_ERR
    global stp, step_frame, WEBSITE_PAGE, v_cal_done, h_cal_done, calib_done
    global end_of_canvas_line, focus_phase, focus_point_coords, FOCUS_CALIBRATED, SHAPE_OF_CUT


    if request.method == 'POST':
        
        if request.form.get('Start') == 'Start':
            if not Started_image_rendering and h_cal_done and v_cal_done and calib_done: 
                ok = True
                if request.form.get('AF'):
                    if FOCUS_CALIBRATED:
                        enable_autofocus = True
                        focus_by_plane()
                    else:
                        ok = False
                        LAST_ERR = "Autofocus not calibrated!"    

                else:
                    enable_autofocus = False
                    
                if request.form.get('POL'):
                    enable_pol_cam = True
                    polarize("off")
                    time.sleep(1)
                else:
                    enable_pol_cam = False

                if ok:
                    frame_sim = np.zeros(((CALIB_CANVAS[1])*SHAPE_OF_CUT[1], (CALIB_CANVAS[0])*SHAPE_OF_CUT[0], 3), np.uint8)
                    frame_pol = np.zeros(((CALIB_CANVAS[1])*SHAPE_OF_CUT[1], (CALIB_CANVAS[0])*SHAPE_OF_CUT[0], 3), np.uint8)

                    FILE_NAME = request.form.get('Fname')
                    coords[0] = 0
                    coords[1] = 0

                    Started_image_rendering = True 

                    WEBSITE_PAGE = 1
                else:
                    pass

            else:
                LAST_ERR = "Render or calibration already started!"
            
        
        elif request.form.get('STOP') == 'STOP' or request.form.get('STOP') == 'STOP/QUIT':
            Started_image_rendering = False
            h_cal_done = True
            v_cal_done = True
            STOP = True
            LAST_ERR = ""
            time.sleep(2)
            STOP = False

            if WEBSITE_PAGE == 1:
                move_to_coord([0,0])

            if WEBSITE_PAGE !=2:
                WEBSITE_PAGE = 0


        elif request.form.get('Canvas') == 'Apply manual dimensions':
            
            if h_cal_done and v_cal_done and not Started_image_rendering:    
                FOCUS_CALIBRATED = False
                x = int(request.form.get('Wcanvas'))
                y = int(request.form.get('Hcanvas'))
                if x > 0 and y > 0:
                    CALIB_CANVAS[0] = x
                    CALIB_CANVAS[1] = y
                else:
                    LAST_ERR = "Not proper Canvas size"
            else:
                LAST_ERR = "Render or calibration already started!"

        elif request.form.get('CamType') == 'Camera Type':
            camera_type = camera_type + 1
            if camera_type > 2:
                camera_type = 0 

        elif request.form.get('Calibration') == 'Width Calib':
            if h_cal_done and v_cal_done and not Started_image_rendering:
                FOCUS_CALIBRATED = False  
                focus()  
                if camera_type != 2:
                    camera_type = 1

                polarize("off")

                pass

                try:
                    vertical_calibration()
                except:
                    print("Error in vertical calibration!")
            else:
                LAST_ERR = "Render or calibration already started!"
            
            camera_type = 0

        elif request.form.get('Calibration') == 'Height Calib':
            if h_cal_done and v_cal_done and not Started_image_rendering:
                FOCUS_CALIBRATED = False
                focus()
                if camera_type != 2:    
                    camera_type = 1

                polarize("off")

                pass
                
                try:
                    horizontal_calibration()
                except:
                    print("Error in horizontal calibration!")

            else:
                LAST_ERR = "Render or calibration already started!"
            
            camera_type = 0

        elif request.form.get('Calibration') == 'Full Calibration':
            if h_cal_done and v_cal_done and not Started_image_rendering: 
                FOCUS_CALIBRATED = False 
                if camera_type != 2:
                    camera_type = 1

                polarize("off")
                
                try:
                    vertical_calibration()
                    full_calibration_mid(True)
                    horizontal_calibration()
                    full_calibration_mid(False)
                except:
                    print("Error in full calibration!")

            else:
                LAST_ERR = "Render or calibration already started!"
    
            camera_type = 0

        elif request.form.get('FMain') == 'Cancel':
            if not Started_image_rendering and h_cal_done and v_cal_done and calib_done:
                STOP = True
                time.sleep(2)
                STOP = False
                WEBSITE_PAGE = 0
                if coords[0] != 0 or coords[1] != 0:
                    move_to_coord([0,0])

            else:
                LAST_ERR = "Render or calibration already started!"            

        elif request.form.get('FMain') == 'Set 0 coord':
            if not Started_image_rendering and h_cal_done and v_cal_done and calib_done:
                coords[0] = 0
                coords[1] = 0
                focus_phase = 0
                FOCUS_CALIBRATED = False
            else:
                LAST_ERR = "Render or calibration already started!"
        
        elif request.form.get('FMain') == 'Set P1':
            if focus_phase == 0:
                if coords[0] >= 0 and coords[1] >= 0 and (coords[0] < CALIB_CANVAS[0]-1 and coords[1] < CALIB_CANVAS[1]-1):

                    focus_point_coords = [[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
                    focus_point_coords[0][0] = coords[0]
                    focus_point_coords[0][1] = coords[1]
                    focus_point_coords[0][2] = 0
                    focus_phase = 1
                    FOCUS_CALIBRATED = False

                else:
                    LAST_ERR = "WRONG P1 PLACEMENT!"

            else:
                LAST_ERR = "Focus Phase ERROR!"
            

        elif request.form.get('FMain') == 'Set P2':
            if focus_phase == 1:
                if coords[0] > focus_point_coords[0][0] and coords[1] > focus_point_coords[0][1] and (coords[0] < CALIB_CANVAS[0] and coords[1] < CALIB_CANVAS[1]):
                    focus_phase = 2
                    LAST_ERR = ""
                    focus()
                    focus_point_coords[3][0] = coords[0]
                    focus_point_coords[3][1] = coords[1]
                    focus_point_coords[3][2] = 0

                    calc_focus_points()
                    calc_foc_plane()

                    if LAST_ERR != "MANUAL FOCUS REQUIRED":
                        FOCUS_CALIBRATED = True
                        WEBSITE_PAGE = 0
                        enable_autofocus = True
                    
                    focus_phase = 0
            
                else:
                    LAST_ERR = "WRONG P2 PLACEMENT!"

            else:
                LAST_ERR = "Focus Phase ERROR!"

        elif request.form.get('FMain') == 'Back to 0 point':
            move_to_coord([0,0])

        elif request.form.get('Calibration') == 'Cutout manual':
            if not Started_image_rendering and h_cal_done and v_cal_done and calib_done:
            
                px = int(request.form.get('PSize'))
                if px > 50 and is_even(px):
                    calib_done = False
                    change_shape_of_cut(px)
                    camera_type = 3
                    overlay_frame_manual()
                    time.sleep(4)
                    camera_type = 0
                    calib_done = True
                else:
                    LAST_ERR = "Cutout has to be even and more than 50!"
            else:
                LAST_ERR = "Render or calibration already started!"

        elif request.form.get('Calibration') == 'Cutout Calib':
            if not Started_image_rendering and h_cal_done and v_cal_done and calib_done:
               
                calib_done = False
                camera_type = 3
                overlay_frame_automatic()
                time.sleep(4)
                camera_type = 0
                calib_done = True
            else:
                LAST_ERR = "Render or calibration already started!"

        elif request.form.get('Calibration') == 'EOL':
            if not Started_image_rendering and focus_phase == 0: 
                end_of_canvas_line = True
                time.sleep(1)
                end_of_canvas_line = False

            else:
                LAST_ERR = "Render already started!"
    
            camera_type = 0

        elif request.form.get('POLM') == 'ON':
            polarize("on")

        elif request.form.get('POLM') == 'OFF':
            polarize("off")

        elif request.form.get('FOC') == 'F':
            FOCUS_CALIBRATED = False
            focus()

        elif request.form.get('FOC') == 'Calibrate AF':
            if not Started_image_rendering and h_cal_done and v_cal_done and calib_done:
                WEBSITE_PAGE = 2
            else:
                LAST_ERR = "Render or calibration already started!"

        else:
            if not Started_image_rendering and h_cal_done and v_cal_done and calib_done:
                stp = int(request.form.get('Steps'))
                if stp > 0:
                    if request.form.get('Stp_type'):
                        step_frame = True
                        if stp > 100:
                            stp = 0
                            LAST_ERR = "Maximum 100 steps!"
                    else:
                        step_frame = False
                        if stp > 3000:
                            stp = 0
                            LAST_ERR = "Maximum 3000 steps!"
                    if WEBSITE_PAGE == 2:
                        step_frame = True

                    manual_move(request.form.get('Move'), amount = stp, step = step_frame)  
            

    elif request.method == 'GET':

        return get_response_page()

    return get_response_page()

@app.route('/video_live')
def video_live():
    return Response(get_frame_live(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/gen_sim')
def gen_sim():
    return Response(get_frame_sim(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/gen_pol')
def gen_pol():
    return Response(get_frame_pol(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/coord_x')
def coord_x():
    global coords
    return "X: " + str(coords[0])

@app.route('/coord_y')
def coord_y():
    global coords
    return "Y: " + str(coords[1])

@app.route('/progress')
def progress():
    global coords, CALIB_CANVAS
    return str(round(coords[1]/CALIB_CANVAS[1], 2)*100).split(".")[0] + " %"

@app.route('/pageID')
def pageID():
    global WEBSITE_PAGE
    return str(WEBSITE_PAGE)


if __name__ == '__main__':

    thread_mot = Thread(target=motor_control)
    thread_mot.daemon = True
    thread_mot.start()
    
    thread_cam = Thread(target=camera_frame_acq)
    thread_cam.daemon = True
    thread_cam.start()

    app.run(debug=False, port=25500, host=IP)
