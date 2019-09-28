import math as ma

# test_data = 400,1280/2 #test input
test_data = 150, 68
hight = 495-114 #mm

#ball parameter
r_ball = 114.8 #mm

#angles
ang_cam_tilt = ma.radians(30)
ang_rev_cam_tilt =  ma.radians(90)-ang_cam_tilt
ang_cam_flare_X = ma.radians(47) #Öffnungswinkel pixy cam
ang_cam_flare_Y = ma.radians(75)

'PIXY Parameter'
#pixy-cam image size in pixel
X_cam_pixel = 800
Y_cam_pixel = 1280
#pixy-cam position parameter
'vorläufige Parameter!!!'
ang_offset = ma.radians(30) #1.0472  # 60 degree
h_cam_offset = 463  # 495-90 #floor to camera-mountings rotation axle
s_cam_offset = 77  # 300 #midpoint robot to camera mounting on x axle
r_cam_rot = 55  #90 #radius of camera rotation circle on xy-level

#absolute position of camera on robot
X_cam_pos = 0
Z_cam_pos = 0
#pixy-cam image parameter
Xmin_image_cam = 0
Xmax_image_cam = 0
Ymin_image_cam = 0
Ymax_image_cam = 0
delta_X_image_cam = 0
delta_Y_image_cam = 0
X_factor_cam = 1
Y_factor_cam = 1
X_ball_ego = 0 #mm
Y_ball_ego = 0 #mm

# pixy-cam position from midpoint robot in relation to tilt angle alpha
def cam_position(alpha, h, s, r):
    print("cam positioN: ", ma.sin(alpha) * r + h, ma.cos(alpha) * r + s)
    return ma.sin(alpha)*r + h, ma.cos(alpha)*r + s

def image_position(Z_cam, X_cam, gamma, delta_X, delta_Y):
    Xmax_image_cam = Z_cam / ma.tan(gamma - (delta_X / 2))
    Xmin_image_cam = Z_cam / ma.tan(gamma + (delta_X / 2))

    print("Xmax, Xmin: ", Xmax_image_cam, Xmin_image_cam)
    Ymax_image_cam = Xmax_image_cam / ma.tan(delta_Y / 2)
    Ymin_image_cam = Xmin_image_cam / ma.tan(delta_Y / 2)

    return Xmax_image_cam - Xmin_image_cam, Ymax_image_cam - Ymin_image_cam

#configuration of pixy-cam results before working
def cam_config():
    Z_cam_pos, X_cam_pos = cam_position(ang_cam_tilt, h_cam_offset, s_cam_offset, r_cam_rot)
    x, y = image_position(Z_cam_pos, X_cam_pos, ang_rev_cam_tilt, ang_cam_flare_X, ang_cam_flare_Y)
    #print("Cam Pos ", Z_cam_pos, X_cam_pos)
    #print("Img Pos ", x, y)

    return x / X_cam_pixel, y / Y_cam_pixel

#process new data from pixy-cam
def cam_process(raw_data):
    x,y = raw_data

    return x * X_factor_cam, y * Y_factor_cam

'MAIN'

X_factor_cam, Y_factor_cam = cam_config()
X_ball_ego, Y_ball_ego = cam_process(test_data)

'DEBUG OUTPUT'
print("Factors: ",X_factor_cam, Y_factor_cam)
print("Länge in mm:", X_ball_ego, Y_ball_ego)
#print(math.tan(math.radians(45)))

