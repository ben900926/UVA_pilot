import cv2

def test():
    arr = [ [0,0,0,0,1],
            [0,1,0,0,0],
            [1,0,0,1,1],
            [0,0,0,1,0],
            [0,1,1,1,0],
            [1,0,1,1,0] ]

    size = (6,5)
    newArr = []
    for i in range(size[0]+2):
        newArr.append(list())
        for j in range(size[1]+2):
            if(i>0 and size[0]>=i and j>0 and size[1]>=j):
                newArr[i].append(arr[i-1][j-1])
            else:
                newArr[i].append(0)

    for i in range(1,size[0]+2):
        for j in range(1,size[1]+2):
            if(newArr[i][j]>0):
    print(newArr)



def main():
    capture = cv2.VideoCapture("car.mp4")
    backSub = cv2.createBackgroundSubtractorMOG2()
    
    if(not capture.isOpened()):
        print("cannot open video")
        return
    while True:
        ret, frame = capture.read()
        if frame is None:
            break
        # back subtract
        fgMask = backSub.apply(frame)

        # remove shadow
        shadowval = backSub.getShadowValue()
        ret, nmask = cv2.threshold(fgMask, shadowval, 255, cv2.THRESH_BINARY)
        #cv2.imshow('Frame',nmask) # shape (240, 320)
        for i in range(nmask.shape[0]):
            l = list()
            for j in range(nmask.shape[1]):
                l.append(nmask[i,j])
            print(l)
        return
        print(nmask.shape) 
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    capture.release()
    cv2.destroyAllWindows()
if __name__ == '__main__':
    test()