# -*- coding: utf-8 -*-
"""
Created on Wed Oct 23 11:06:48 2019

@author: riyak
"""

import math
import time
import copy

start_time = time.time()
file1 = open("input.txt","r")
file2 = open("output.txt","w+")
moveType = file1.readline().strip()
pieceType = file1.readline().strip()
timeLeft = file1.readline().strip()
Depth =0
move_list=[]
SMALL_NUM = 2**-31

class Node:
    board=[]
    value=0
    children=None
    nodeType = ""
    minMax = 0 #1 for max, 0 for min
    parent_x=0
    parent_y=0
    depth=0
    moved_to_x=-1
    moved_to_y=-1
    possible_moves = []
    
    def __init__(self, board, value, children, nodeType, minMax, parent_x, parent_y, depth, moved_to_x, moved_to_y, pos): #nodes_visited,
        self.board = board
        self.value = value
        self.children = children
        self.nodeType = nodeType
        self.minMax = minMax
        self.parent_x = parent_x
        self.parent_y = parent_y
        self.depth = depth
        self.moved_to_x = moved_to_x
        self.moved_to_y = moved_to_y
        self.possible_moves = pos
    

board = []
for i in file1:
    l = []
    for ch in range(16):
        l.append(i[ch])
    board.append(l)
    
blackHome = [[0,0], [1,0], [2,0], [3,0], [4,0],
             [0,1], [1,1], [2,1], [3,1], [4,1],
             [0,2], [1,2], [2,2], [3,2],
             [0,3], [1,3], [2,3],
             [0,4], [1,4]
             ]

whiteHome = [[14,11], [15,11],
             [13,12], [14,12], [15,12],
             [12,13], [13,13], [14,13], [15,13],
             [11,14], [12,14], [13,14], [14,14], [15,14],
             [11,15], [12,15], [13,15], [14,15], [15,15]
             ]
def generate_jump_path2(fromx, fromy, tox, toy, home):
#    print(fromx, fromy, tox, toy)
    jumpTuple=[]
    jump_recursive2(fromx, fromy, home, [], board, jumpTuple, tox, toy)
#    print(jumpTuple)
    a = [[tox,toy]]
    p = [item for item in jumpTuple if item[1][0]==tox and item[1][1]==toy][0]
    s = p[1]
    a.append(p[0])
#    print(a, "kjflsdjflsjflsjflsjflsjfljsdflsjfldsjfljsf")
#    print(s, "lolo")
    while s[0]!=fromx or s[1]!=fromy:
        p = [item for item in jumpTuple if item[1][0]==s[0] and item[1][1]==s[1]]
        s = p[0][0]
        if s not in a:
            a.append(s)
#        print(s, "acha", [fromx, fromy])
    a.reverse()
#    print(a)
    return a
    
def jump_recursive2(x, y, home, nodes_visited, board, jumpTuple, tox, toy):
    jumps=[]
    for i in range(1, -2, -1):
            if 0<=x+i<16:
                for j in range(1, -2, -1):
                    if 0<=y+j<16:
                        if [i+x,j+y]!=[x,y]:
                            if board[j+y][i+x]!='.':
                                jumpToX=x + i*2
                                jumpToY=y + j*2
                                if 0<=jumpToX<16 and 0<=jumpToY<16:
                                    if board[jumpToY][jumpToX]=='.' and [jumpToX,jumpToY] not in nodes_visited:#[jumpToX, jumpToY] not in nodes_visited:
                                        nodes_visited.append([x,y])
                                        jumps.append([jumpToX,jumpToY])
                                        jumpTuple.append(([x,y],[jumpToX, jumpToY]))
                                        if jumpToX==tox and jumpToY==toy:
                                            return jumps
                                        future_jumps = jump_recursive2(jumpToX, jumpToY, home, nodes_visited, board, jumpTuple, tox, toy)
                                        jumps.extend(future_jumps)
#    print(jumps)
    return jumps #, nodes_visited
    
    
def game():
    if pieceType=='WHITE':
        piece = 'W'
    else:
        piece = 'B'
    root=Node(board, float('-inf'), [], piece, 1, -1, -1, 0, -1, -1, []) #float('-inf')
    minmax_algo(root)
    
    

def minmax_algo(node):
    v = max_value(node, float('-inf'), float('inf')) #-2**31, 2**31-1
#    print("riya",v)   
    for i in node.children:
#        print(i.value, "hurray ", i.moved_to_x, i.moved_to_y)
#        print(i.value, "x y ", i.parent_x, i.parent_y)
        #print(i.board)
        if i.value==v:
#            print("MinMax",v)
            if abs(i.parent_x-i.moved_to_x)>1 or abs(i.parent_y-i.moved_to_y)>1:
                home = whiteHome if node.nodeType=='W' else blackHome
                path_list = generate_jump_path2(i.parent_x, i.parent_y, i.moved_to_x, i.moved_to_y, home)
                #it is a jump
#                path_list = generate_jump_path(i.parent_x, i.parent_y, i.moved_to_x, i.moved_to_y, i.possible_moves)
                for i in range(len(path_list)-2):
                    path="J "+str(path_list[i][0])+","+str(path_list[i][1])+" "+str(path_list[i+1][0])+","+str(path_list[i+1][1])
                    file2.write(path+"\n")
#                    print(path)
                l = len(path_list)-2
#                print("J "+str(path_list[l][0])+","+str(path_list[l][1])+" "+str(path_list[l+1][0])+","+str(path_list[l+1][1]))
                file2.write("J "+str(path_list[l][0])+","+str(path_list[l][1])+" "+str(path_list[l+1][0])+","+str(path_list[l+1][1]))
                file2.close()
                return v
                
            else:
                path="E "+str(i.parent_x)+","+str(i.parent_y)+" "+str(i.moved_to_x)+","+str(i.moved_to_y)
                file2.write(path)
#                print(path)
                file2.close()
                return v
    
def max_value(state, alpha, beta):
    global Depth
#    if state.nodeType=='W':
#        goal = whiteHome
#    else:
#        goal = blackHome
    if state.depth==Depth :
        state.value = utility(state)
        return state.value
    v = float('-inf') #-2**31
    #state.children = 
    create_children(state)
#    print(Depth)
    if state.depth==0 and len(state.children)>60:
        Depth=1
#        print(Depth)
    for obj in state.children:
        v = max(v, min_value(obj, alpha, beta))
        if v>=beta:
            state.value = v
            return v
        alpha = max(alpha, v)
    state.value = v
    return v
    
def min_value(state, alpha, beta):
    #might need to write depth==1 logic for utility if
    if state.depth==Depth :
        state.value = utility(state)
        return state.value
    v = float('inf') #2**31-1
    #state.children = 
    create_children(state)
    for obj in state.children:
        
        
        v = min(v, max_value(obj, alpha, beta))
        if state.depth==1 :
#            print(obj.parent_y, obj.parent_x, obj.moved_to_x, obj)
#            print("depth 1")
            if (obj.nodeType=='B' and [state.parent_y, state.parent_x] in blackHome and [state.moved_to_y, state.moved_to_x] not in blackHome) or (obj.nodeType=='W' and [state.parent_y, state.parent_x] in whiteHome and [state.moved_to_y, state.moved_to_x] not in whiteHome):
                v=v*100
            if (obj.nodeType=='B' and [state.parent_y, state.parent_x] in blackHome and [state.moved_to_y, state.moved_to_x] in blackHome) or (obj.nodeType=='W' and [state.parent_y, state.parent_x] in whiteHome and [state.moved_to_y, state.moved_to_x] in whiteHome):
                v=v*70
            if (obj.nodeType=='W' and [state.parent_y, state.parent_x] not in blackHome and [state.moved_to_y, state.moved_to_x] in blackHome) or (obj.nodeType=='B' and [state.parent_y, state.parent_x] not in whiteHome and [state.moved_to_y, state.moved_to_x] in whiteHome):
                v=v*50
            if (obj.nodeType=='B' and [state.parent_y, state.parent_x] in whiteHome and [state.moved_to_y, state.moved_to_x] in whiteHome) or (obj.nodeType=='W' and [state.parent_y, state.parent_x] in blackHome and [state.moved_to_y, state.moved_to_x] in blackHome):
                v=v*20
        if v<=alpha: 
            state.value = v
            return v
        beta = min(beta, v) 
    
    state.value = v
    return v

def generate_nodes(node, pawns):
    if node.nodeType=='W':
        homeType = whiteHome
        goalType = blackHome
        childColor = 'B'
    elif node.nodeType=='B':
        homeType = blackHome
        childColor = 'W'
        goalType = whiteHome
        
    if node.minMax==1:
        val = float('-inf') #-2**31
        minmaxType = 0
    elif node.minMax==0:
        val = float('inf') #2**31-1 
        minmaxType = 1
        
    i=0
    while i<len(pawns):
        if(pawns[i] not in goalType):
            pawnFromX = pawns[i][1]
            pawnFromY = pawns[i][0]
            possible = get_neighbors(pawnFromX, pawnFromY, homeType, goalType, node.board) 
            #print(possible, "possible")
            for p in possible:
                #print(p)
                newBoard=copy.deepcopy(node.board)
                #print(possible)
                l = Node(newBoard, val, [], childColor, minmaxType, pawnFromX, pawnFromY, node.depth+1, p[0], p[1], possible)#instead of childcolor it should be node.nodeType #,node_visited
                l.board[pawnFromY][pawnFromX] = "."
                l.board[p[1]][p[0]] = node.nodeType
                node.children.append(l)
        elif pawns[i] in goalType:
            pawnFromX = pawns[i][1]
            pawnFromY = pawns[i][0]
            possible = get_neighbors(pawnFromX, pawnFromY, homeType, goalType, node.board) #,nodes_visited
            #print(possible, "possible")
            for p in possible:
                #print(p)
                newBoard=copy.deepcopy(node.board)
                #print(possible)
                l = Node(newBoard, val, [], childColor, minmaxType, pawnFromX, pawnFromY, node.depth+1, p[0], p[1], possible)#instead of childcolor it should be node.nodeType #,node_visited
                l.board[pawnFromY][pawnFromX] = "."
                l.board[p[1]][p[0]] = node.nodeType
                node.children.append(l)
        i+=1
    return  node.children

def create_children(node):
    if node.nodeType=='W':
        homeType = whiteHome
    elif node.nodeType=='B':
        homeType = blackHome
        
    pawnsInHome=[]
    pawnPositions=[]
    for i in range(16):
        for j in range(16):
            if node.board[j][i]==node.nodeType:
                if [j,i] in homeType: #nodeType is 'W' or 'B'
                    pawnsInHome.append([j,i])
                else:
                    pawnPositions.append([j,i])
    generate_nodes(node, pawnsInHome)
    if len(node.children)>0:
        return node.children
    else:
        generate_nodes(node,pawnPositions)
        return node.children

def utility(node):    
    white_value=0
    black_value=0
    for i in range(16):
        for j in range(16):
            if node.board[j][i]=='W' :
                goalList = blackHome
                distances = [euclidean_distance(i,j,goal[1],goal[0]) for goal in blackHome if node.board[goal[1]][goal[0]]!='W']
#                white_value += max(distances) if len(distances)>0 else -100
                if len(distances)>0:
                    white_value += max(distances)
                elif [node.moved_to_y,node.moved_to_x] in goalList and [node.parent_y, node.parent_x] in whiteHome:
                    white_value = SMALL_NUM
                else:
                    white_value = -100#float('-inf')
            elif node.board[j][i]=='B' :
                goalList = whiteHome
                
                distances = [euclidean_distance(i,j,goal[1],goal[0]) for goal in whiteHome if node.board[goal[1]][goal[0]]!='B']
                if len(distances)>0:
                    black_value += max(distances)
                elif [node.moved_to_y,node.moved_to_x] in goalList and [node.parent_y, node.parent_x] in blackHome:
                    black_value = SMALL_NUM
                else:
                    black_value = -100
#                    black_value += max(distances) if len(distances) else -100#float('-inf')
            
    value = white_value/black_value if node.nodeType=='W' else black_value/white_value  
#    print(value)
    return value
    
    
def get_neighbors(x, y, home, goal, board):
    possibleMoves = []
    jumps = jump_recursive(x, y, home, [], board)
    for j in jumps:
        if j in home:
            jumps.remove(j)
    possibleMoves.extend(jumps)
    for i in range(x+1, x-2, -1):
            if 0<=i<16:
                for j in range(y+1, y-2, -1):
                    if 0<=j<16:
                        if [i,j]!=[x,y]:
                            if is_valid_move(i, j, home, board):
                                possibleMoves.append([i,j])
                            if [j,i] in home:
                                if valid_move_h_d(x,y,i,j, board, 0):
                                    possibleMoves.append([i,j])  
                            if [y,x] in goal and [j,i] in goal and valid_move_h_d(x,y,i,j,board, 1):
                                possibleMoves.append([i,j])
                                
    #print(possibleMoves)
    return possibleMoves #,nodes_visited

def jump_recursive(x, y, home, nodes_visited, board):
    jumps=[]
#    for i in range(-1, 2):
#            if 0<=x+i<16:
#                for j in range(-1, 2):
    for i in range(1, -2, -1):
            if 0<=x+i<16:
                for j in range(1, -2, -1):
                    if 0<=y+j<16:
                        if [i+x,j+y]!=[x,y]:
                            if board[j+y][i+x]!='.':#isAnObstacle(i+x, j+y, home, nodes_visited):#, obstacles):
                                #check if saamne waala is empty, then only do add to jumps and all
                                jumpToX=x + i*2
                                jumpToY=y + j*2
                                if 0<=jumpToX<16 and 0<=jumpToY<16:
                                    if board[jumpToY][jumpToX]=='.' and [jumpToX,jumpToY] not in nodes_visited:#[jumpToX, jumpToY] not in nodes_visited:
                                        nodes_visited.append([x,y])
#                                        print(board[x][y])
#                                        if jumpToX==10 and jumpToY==13 and x==12 and y==13:
#                                            print("yeah yeah")
                                        #append to jumps
                                        jumps.append([jumpToX,jumpToY])
                                        #get future jumps
                                        #append to jumps
                                        future_jumps = jump_recursive(jumpToX, jumpToY, home, nodes_visited, board)
                                        jumps.extend(future_jumps)
    return jumps #, nodes_visited
    
def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)  
    
def is_valid_move(x, y, home, board):
    if board[y][x]=='.' and [y,x] not in home:
        return True
    
def valid_move_h_d(x, y, m, n, board, flag):
    if flag==0:
        if board[y][x]=='W' and abs(x-15)+abs(y-15)<abs(m-15)+abs(n-15) and board[n][m]=='.':
            return True
        elif board[y][x]=='B' and x+y<m+n and board[n][m]=='.':
            return True
    else:
        if board[y][x]=='B' and abs(x-15)+abs(y-15)>abs(m-15)+abs(n-15) and board[n][m]=='.':
            return True
        elif board[y][x]=='W' and x+y>m+n and board[n][m]=='.':
            return True
     

#print(moveType)
#print(pieceType)
#print(timeLeft)
#print(board)
if moveType.lower()=='single':
    Depth=1
    game()
if moveType.lower()=='game':
    Depth = 3 if float(timeLeft)>5 else 1
#    print(Depth)
    game()
#print(time.time()-start_time)