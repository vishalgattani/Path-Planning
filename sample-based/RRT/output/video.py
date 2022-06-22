import cv2
import sys
import os
import glob 
import re 

def sorted_nicely( l ): 
    """ Sort the given iterable in the way that humans expect.""" 
    convert = lambda text: int(text) if text.isdigit() else text 
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ] 
    return sorted(l, key = alphanum_key)


image_folder = '.'
fps = 60
video_name = "rrt_"+str(fps)+"fps.avi"


images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
# for x in sorted_nicely(images):
#     print(x)
images = sorted_nicely(images)
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape
print(height,"x",width,"-",len(images))


fourcc = cv2.VideoWriter_fourcc(*'mp4v') 

# out = cv2.VideoWriter('video.avi', fourcc, 1.0, img_arr[0].shape)
video = cv2.VideoWriter(video_name, fourcc, fps, (width,height))

for image in images:
    video.write(cv2.imread(os.path.join(image_folder, image)))

cv2.destroyAllWindows()
print("Video File: ",video_name)