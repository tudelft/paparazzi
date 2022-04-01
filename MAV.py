import numpy as np


drone_pos = np.array([2.5,0]) #current drone position, middle of cyberzoo

arena_length = 5 #lenght of the cyberzoo??

#random matrix with 3 obstacles
#coordinate system: z poining forward, x pointing to the right, all reference drone
#format assumed: [xleft_edge,zleft_edge,xright_edge,zright_edge]
#assumed obstacles are ordered from left to right as rows in matrix

obstacles = np.array([[-1.7,2.1,-1.4,2.1],
                      [-0.5,4.5,-0.1,4.8],
                      [0.3,3.6,0.6,3.4]])

Nobst = obstacles.shape[0]


headings = np.zeros(Nobst+1)

heading_boundary = 10 #the manual left and right boundary of the heading

heading_start = (heading_boundary-90)/180*np.pi
heading_end = (90-heading_boundary)/180*np.pi

dronewidth = 0.50 #including margin, TBD


for i in range(Nobst+1):
    #the heading left of most left obstacle
    if i == 0:
        headings[i] = (heading_start+np.tan(obstacles[i,0]/obstacles[i,1]))/2
        #set entry to false if the drone cannot fit through
        if (np.abs(np.cos(headings[i])*(obstacles[i,0]-drone_pos[0]))<dronewidth):
            headings[i] = False
    #the heading right of most right obstacle
    elif i == Nobst:
        headings[i]= (heading_end + np.tan(obstacles[i-1, 2]/obstacles[i-1, 3])) / 2
        if (np.abs(np.cos(headings[i])*(arena_length-drone_pos[0]-obstacles[i-1,2]))<dronewidth):
            headings[i] = False
    #heading in between obstacles
    else:
        headings[i] = (np.tan(obstacles[i,0]/obstacles[i,1])+np.tan(obstacles[i-1,2]/obstacles[i-1,3]))/2
        if (np.abs(np.cos(headings[i])*(obstacles[i,0]-obstacles[i-1,2]))<dronewidth):
            headings[i] = False

#remove "false" headings from possible heading list
headings = np.delete(headings,np.where(headings == 0)[0])

#calculate path length
paths = np.zeros(len(headings))
for j in range(len(headings)):
    if headings[j]>0:
        #check if the drone moves to opposite or side wall
        if (np.tan(headings[j])*arena_length<(arena_length-drone_pos[0])):
            #for this case drone moves to opposite wall
            paths[j] = arena_length/np.cos(headings[j])
        else:
            #drone moves to side wall
            paths[j] = np.sin(headings[j])*(arena_length-drone_pos[0])
    else:
        if (np.tan(np.abs(headings[j]*arena_length)<drone_pos[0])):
            #opposite wall
            paths[j] = arena_length/np.cos(np.abs(headings[j]))
        else:
            paths[j] = arena_length/np.sin(np.abs(headings[j]))

maxpath_index = np.where(paths == np.amax(paths))[0]

print("Longest path = " , paths[maxpath_index], 'm, with heading', headings[maxpath_index]*180/np.pi, "degrees")