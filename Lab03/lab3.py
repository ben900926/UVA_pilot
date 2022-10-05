import cv2
import numpy as np
import json
def test(arr=None,T=150):
    
    """arr = [ [0,0,1,0,1],
            [0,1,0,0,1],
            [1,0,1,1,1],
            [0,0,0,0,0],
            [0,1,1,1,0],
            [1,0,0,1,0] ]"""

    #size = (6,5)
    size = (arr.shape[0],arr.shape[1])
    newArr = []
    for i in range(size[0]+2):
        newArr.append(list())
        for j in range(size[1]+2):
            if(i>0 and size[0]>=i and j>0 and size[1]>=j):
                newArr[i].append(arr[i-1][j-1])
            else:
                newArr[i].append(0)

    # first pass
    eqi_list = []
    label = 1
    for i in range(1,size[0]+2):
        for j in range(1,size[1]+2):
            if(newArr[i][j]!=0):
                # check for different nei value
                neighbors = [newArr[i][j-1],newArr[i-1][j-1],newArr[i-1][j],newArr[i-1][j+1]]
                non_zero_nei = -1
                # check for first non zero
                for k in range(len(neighbors)):
                    if(neighbors[k]!=0):
                        non_zero_nei = k
                        break
                nei_value = neighbors[non_zero_nei]
                for n in neighbors[non_zero_nei+1:]:
                # check for different value
                    if(n!=0 and n!=nei_value):
                        big = max(n,nei_value)
                        small = min(n,nei_value)
                        if(small==0):
                            continue
                        if((big,small) not in eqi_list):
                            eqi_list.append((big,small))
                
                # label assign
                min_value = 256
                if(newArr[i][j-1]!=0):
                    min_value = min(min_value,newArr[i][j-1])
                if(newArr[i-1][j-1]!=0):
                    min_value = min(min_value,newArr[i-1][j-1])
                if(newArr[i-1][j]!=0):
                    min_value = min(min_value,newArr[i-1][j])
                if(newArr[i-1][j+1]!=0):
                    min_value = min(min_value,newArr[i-1][j+1])
                
                if(min_value >= 256):
                    newArr[i][j] = label
                    label += 1
                else:
                    newArr[i][j] = min_value
        #print(newArr[i])

    # remove padding
    arr = list()
    for i in range(size[0]):
        arr.append(list())
        for j in range(size[1]):
            arr[i].append(newArr[i+1][j+1])

    # label replacing
    for i in range(size[0]):
        for j in range(size[1]):
            if(arr[i][j]!=0):
                for k in eqi_list:
                    if(arr[i][j]==k[0]):
                        arr[i][j] = k[1]
        #print(arr[i])

    #print(eqi_list)
    
    size_list = list()
    # count for area
    for i in range(label):
        size_list.append(0)
    for i in range(size[0]):
        for j in range(size[1]):
            if(arr[i][j]!=0):
                size_list[arr[i][j]]+=1

    edge_dict = dict() # store edges
    # check for area
    for i in range(size[0]):
        for j in range(size[1]):
            if(size_list[arr[i][j]] > T):
                label = arr[i][j]
                if(label not in edge_dict):
                    edge_dict[label] = [10000,10000, 0,0]
                (edge_dict[label][0]) = min(edge_dict[label][0],i)
                (edge_dict[label][1]) = min(edge_dict[label][1],j)
                (edge_dict[label][2]) = max(edge_dict[label][2],i)
                (edge_dict[label][3]) = max(edge_dict[label][3],j)
    
    return edge_dict

def preprocess(to_save=True):
    i = 0
    edge_dicts = list()
    capture = cv2.VideoCapture("car.mp4")
    backSub = cv2.createBackgroundSubtractorMOG2()
    
    if(not capture.isOpened()):
        print("cannot open video")
        return
    with open('cars.txt', 'w') as f:
        while True:
            i+=1
            ret, frame = capture.read()
            if frame is None:
                break
            # back subtract
            fgMask = backSub.apply(frame)

            # remove shadow
            shadowval = backSub.getShadowValue()
            ret, nmask = cv2.threshold(fgMask, shadowval, 255, cv2.THRESH_BINARY)
            d = test(nmask)
            if(i%10==0):
                print(i)
            edge_dicts.append(d)
            #print(edge_dicts)

        if(to_save):
            for j in range(len(edge_dicts)):
                f.write("\n")
                for d in edge_dicts[j]:
                    edges = edge_dicts[j][d]
                    for e in edges:
                        f.write(str(e)+' ')

    return edge_dicts

def main(edge_dicts):
    #print(edge_dicts)
    capture = cv2.VideoCapture("car.mp4")
    backSub = cv2.createBackgroundSubtractorMOG2()
    
    if(not capture.isOpened()):
        print("cannot open video")
        return
    i = 0
    while True:
        ret, frame = capture.read()
        if frame is None:
            break
        # draw rec
        red_color = (0, 255, 0) # BGR

        #for edge_d in edge_dicts:
        if(i>=len(edge_dicts)):
            i = 0

        # [224, 130, 239, 166]
        for j in range(0,len(edge_dicts[i]),4):
            #edges = edge_dicts[i][d]
            #print((edges[1][0], edges[0][1]), (edges[1][0], edges[1][1]))
            cv2.rectangle(frame, (edge_dicts[i][j+1], edge_dicts[i][j+0]), (edge_dicts[i][j+3], edge_dicts[i][j+2]), red_color, 3, cv2.LINE_AA)
        cv2.imshow('Frame',frame) # shape (240, 320)
        
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
        i+=1
    capture.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # d = preprocess()
    # about 740 frames
    d = list()
    # load file
    with open("cars.txt",'r') as f:
        for line in f:
            line = line.split(' ')
            line = line[:-1]
            for i,l in enumerate(line):
                line[i] = int(l)
            d.append(line)
    
    #print(d)
    main(d)
