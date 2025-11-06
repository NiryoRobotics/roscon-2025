import cv2, os, sys, time
name=sys.argv[1]
os.makedirs(name,exist_ok=True)
cap=cv2.VideoCapture(0)
for i in range(50):
    ret,frame=cap.read()
    if not ret: continue
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    lower_red1,upper_red1=(0,100,100),(10,255,255)
    lower_red2,upper_red2=(160,100,100),(179,255,255)
    lower_blue,upper_blue=(100,100,100),(140,255,255)
    mask1=cv2.inRange(hsv,lower_red1,upper_red1)
    mask2=cv2.inRange(hsv,lower_red2,upper_red2)
    mask3=cv2.inRange(hsv,lower_blue,upper_blue)
    mask=cv2.bitwise_or(mask1,cv2.bitwise_or(mask2,mask3))
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    gray=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)
    color=cv2.bitwise_and(frame,frame,mask=mask)
    inv_mask=cv2.bitwise_not(mask)
    bw=cv2.bitwise_and(gray,gray,mask=inv_mask)
    result=cv2.add(color,bw)
    filename=f"{name}/{name}_img_{i:03d}.png"
    cv2.imwrite(filename,result)
    time.sleep(1)
cap.release()
