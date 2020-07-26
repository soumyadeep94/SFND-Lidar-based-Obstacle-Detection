import cv2
import numpy as np
import os

image_folder = '.' # make sure to use your folder 
video_name = 'RenderedVideo.avi'
os.chdir("saveRender") 
  
images = [img for img in os.listdir(image_folder)] 
 
# Array images should only consider 
# the image files ignoring others if any 
#print(images)  

frame = cv2.imread(os.path.join(image_folder, images[0])) 

# setting the frame width, height width 
# the width, height of first image 
fourcc = cv2.VideoWriter_fourcc(*'MP42')
FPS = 4
height, width, layers = frame.shape   

video = cv2.VideoWriter(video_name, fourcc, float(FPS), (width, height))  

# Appending the images to the video one by one 
for image in images:  
    video.write(cv2.imread(os.path.join(image_folder, image)))  
  
# Deallocating memories taken for window creation 
cv2.destroyAllWindows()  
video.release();

