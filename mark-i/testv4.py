import SimpleCV
cam = SimpleCV.Camera()

while True:
    img = cam.getImage()
    img.show()
