import numpy as np
import random
import plotly as py
import cv2 as cv
import rospy
import math
import time
from shapely.geometry import Point
from plotly import graph_objs as go
from rtree import index
from operator import itemgetter
from tqdm import tqdm
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

# --------------------- Global Variables --------------------- #

start_end=[False,False] #start and end points selected flag
x_init=None #start point
x_goal=None #goal point
invert_y=0 #inverted y coordinate
scale=1 #scale of the map

# --------------------- Global Variables --------------------- #
# --------------------- Utility Functions --------------------- #

def distance(a, b):
    distance = np.linalg.norm(np.array(b) - np.array(a))
    return distance

def interpolated_points(start, end, r):
    d = distance(start, end)
    n_points = int(np.ceil(d / r))
    if n_points > 1:
        step = d / (n_points - 1)
        for i in range(n_points):
            next_point = steer(start, end, i * step)
            yield next_point

def steer(start, goal, d):
    start, end = np.array(start), np.array(goal)
    v = end - start
    u = v / (np.sqrt(np.sum(v ** 2)))
    steered_point = start + u * d
    return tuple(steered_point)

def path_cost(E, a, b):
    cost = 0
    while not b == a:
        p = E[b]
        cost += distance(b, p)
        b = p

    return cost

def create_map(case=1):
    global scale
    check_input=True
    while check_input:
        clearance=int(input("Please provide the clearance value(mm):"))
        if clearance<0:
            print("Please enter a positive value\n")
            continue
        clearance/=10
        clearance+=8
        clearance=int(clearance)
        check_input=False
        
    print("\rTotal clearance value(cm):",clearance)

    if case==1:
        scale=200
        dimensions = np.array([(0, 600), (0, 400)])  # dimensions of Search Space
        map_image=np.ones((400,600),np.uint8)*255
        
        obstacles=[(140-clearance,75-clearance,170+clearance,105+clearance),(140-clearance,185-clearance,170+clearance,215+clearance),
            (140-clearance,295-clearance,170+clearance,325+clearance),(290-clearance,0,320+clearance,30+clearance),
            (290-clearance,120-clearance,320+clearance,150+clearance),(290-clearance,250-clearance,320+clearance,280+clearance),
            (290-clearance,370-clearance,320+clearance,400),(440-clearance,75-clearance,470+clearance,105+clearance),
            (440-clearance,185-clearance,470+clearance,215+clearance),(440-clearance,295-clearance,470+clearance,325+clearance)]
            
        for obstacle in obstacles:
            cv.rectangle(map_image,(obstacle[0],obstacle[1]),(obstacle[2],obstacle[3]),(0,0,0),-1)

    elif case ==2:
        scale=100
        map_image=np.ones((200,600),np.uint8)*255
        dimensions = np.array([(0, 600), (0, 200)])  # dimensions of Search Space
        
        obstacles=[(250-clearance, 0,265+clearance, 125+clearance),(150-clearance,75-clearance, 165+clearance, 200-clearance),
                (0, 0, 600,clearance),(0, 0, clearance, 200),(0, 200-clearance, 600, 200),(600-clearance, 0, 600, 200),(400,110, 50 + clearance)]
        for obstacle in obstacles:
            if len(obstacle) == 4:
                cv.rectangle(map_image, (obstacle[0], 200-obstacle[1]), (obstacle[2], 200-obstacle[3]), (0, 0, 0), -1)
            elif len(obstacle) == 3:
                cv.circle(map_image, (obstacle[0], 200-obstacle[1]), obstacle[2], (0, 0, 0), -1)

    return dimensions,obstacles,map_image

def start_goal_selector(event, x, y, flags, param):
    global x_init,x_goal,start_end
    point=None
    if event == cv.EVENT_LBUTTONDOWN:
        point=(x,invert_y-y)
        # point=(x,map_image.shape[1]-y)
    
    if event == cv.EVENT_MOUSEMOVE:
        if start_end[0]==False:
            print("Hover over the start point and press left click",end="\r")
        elif start_end[1]==False:
            print("Hover over the goal point and press left click",end="\r")


    if start_end[0]==False and point is not None:
        x_init=point
        start_end[0]=True
        print("\nStart:",x_init)
    elif start_end[1]==False and point is not None:
        x_goal=point
        start_end[1]=True
        print("\nGoal:",x_goal)


def get_orientation(msg, current):
    current['x'], current['y'] = msg.pose.pose.position.x, msg.pose.pose.position.y
    _, _, current['yaw'] = euler_from_quaternion((msg.pose.pose.orientation.x,
                                                  msg.pose.pose.orientation.y,
                                                  msg.pose.pose.orientation.z,
                                                  msg.pose.pose.orientation.w))

def move_robot(path, competition=False):
    path = [[500,370],[602,370]]
    current = {'x': 0, 'y': 0, 'yaw': 0}    # Dictionary to store the current position of the robot
    odom_sub = rospy.Subscriber('/odom', Odometry, get_orientation, callback_args=current)  # Subscribe to the odom topic to get the current position of the robot
    
    print("\nTraversing the path:")
    for point in tqdm(path):
        goal_x, goal_y = point
        # goal_y=200-goal_y
        # goal_x-=1
        goal_y-=invert_y
        goal_x/=scale
        goal_y/=scale

        pub, rate = rospy.Publisher('/cmd_vel', Twist, queue_size=10), rospy.Rate(10)   # Create a publisher and rate object
        reached_goal, Velocity= False, Twist()

        while not reached_goal and not rospy.is_shutdown():
            Deltax, Deltay = goal_x - current['x'], goal_y - current['y']   # Calculate delta x and delta y
            goal_angle, distance_to_goal = math.atan2(Deltay, Deltax), math.sqrt(Deltax**2 + Deltay**2) # Calculate goal angle and distance to goal

            if abs(goal_angle - current['yaw']) > 0.1:  # If the robot is not facing the goal, rotate
                Velocity.linear.x, Velocity.angular.z = 0.0, 2*(goal_angle - current['yaw'])  # Rotate towards the goal
            else:
                Velocity.linear.x, Velocity.angular.z = min(3, distance_to_goal), 0.0   # Move towards the goal
                reached_goal = distance_to_goal < 0.05  # Check if the robot has reached the goal

            pub.publish(Velocity)
            rate.sleep()

    if competition == True:
        Velocity.linear.x=2.5
        t=time.time()
        while time.time()-t<3:
            pub.publish(Velocity)
        # time.sleep(3)
        Velocity.linear.x=0
        pub.publish(Velocity)


    Velocity.linear.x=0
    Velocity.angular.z=0
    pub.publish(Velocity)

    print("\nReached the end node")
    

# --------------------- Utility Functions --------------------- #
# --------------------- Tree --------------------- #

class Tree:
    def __init__(self, X):
        p = index.Property()
        p.dimension = X.dimensions
        self.V = index.Index(interleaved=True, properties=p)  # vertices in an rtree
        self.V_count = 0
        self.E = {}  # edges in form E[child] = parent

# --------------------- Tree --------------------- #
# --------------------- Search Space --------------------- #

class OccupancyGrid:
    def __init__(self, dimension_lengths, O=None):
        if len(dimension_lengths) < 2:
            raise Exception("Must have at least 2 dimensions")
        self.dimensions = len(dimension_lengths)  # number of dimensions
        self.dimension_lengths = dimension_lengths  # length of each dimension
        p = index.Property()
        p.dimension = self.dimensions
        if O is None:
            self.obs = index.Index(interleaved=True, properties=p)
        else:
            # r-tree representation of obstacles
            self.obs = index.Index(self.obstacle_generator(O), interleaved=True, properties=p)

    def check_free(self, x):
        return self.obs.count(x) == 0

    def sample_free(self):
        while True:  # sample until not inside of an obstacle
            x = self.sample()
            if self.check_free(x):
                return x

    def collision_free(self, start, end, r):
        points = interpolated_points(start, end, r)
        coll_free = all(map(self.check_free, points))
        return coll_free

    def sample(self):
        x = np.random.uniform(self.dimension_lengths[:, 0], self.dimension_lengths[:, 1])
        return tuple(x)

    def obstacle_generator(self,obstacles):
        for obstacle in obstacles:
            id = random.randint(0, 100000)
            if len(obstacle) ==3:
                obstacle=self.add_obstacle(obstacle)
            yield (id, obstacle, obstacle)

    def add_obstacle(self,obs):
        x,y,radius = obs
        obstacle = Point(x, y).buffer(radius)
        xmin, ymin, xmax, ymax = obstacle.bounds
        return (xmin,ymin,xmax,ymax)
            
# --------------------- Search Space --------------------- #
# --------------------- Plot --------------------- #

class Plot:
    colors = ['darkblue', 'teal']
    def __init__(self, filename):
        self.filename = filename + ".html"
        self.data = []
        self.layout = {'title': 'Plot',
                       'showlegend': False
                       }

        self.fig = {'data': self.data,
                    'layout': self.layout}

    def plot_tree(self, trees):
        for i, tree in enumerate(trees):
            for start, end in tree.E.items():
                if end is not None:
                    trace = go.Scatter(
                        x=[start[0], end[0]],
                        y=[start[1], end[1]],
                        line=dict(
                            color=self.colors[i]
                        ),
                        mode="lines"
                    )
                    self.data.append(trace)

    def plot_obstacles(self, O):
        self.layout['shapes'] = []
        for O_i in O:
            if len(O_i) == 3: # circle
                self.layout['shapes'].append(
                    {
                        'type': 'circle',
                        'x0': O_i[0] - O_i[2],
                        'y0': O_i[1] - O_i[2],
                        'x1': O_i[0] + O_i[2],
                        'y1': O_i[1] + O_i[2],
                        'line': {
                            'color': 'purple',
                            'width': 4,
                        },
                        'fillcolor': 'purple',
                        'opacity': 0.70
                    },
                )
            else:
                self.layout['shapes'].append(
                    {
                        'type': 'rect',
                        'x0': O_i[0],
                        'y0': O_i[1],
                        'x1': O_i[2],
                        'y1': O_i[3],
                        'line': {
                            'color': 'purple',
                            'width': 4,
                        },
                        'fillcolor': 'purple',
                        'opacity': 0.70
                    },
                )

    def plot_path(self, path):
        x, y = [], []
        for i in path:
            x.append(i[0])
            y.append(i[1])
        trace = go.Scatter(
            x=x,
            y=y,
            line=dict(
                color="red",
                width=4
            ),
            mode="lines"
        )
        self.data.append(trace)

    def plot_start_end(self, point, color):
        trace = go.Scatter(
            x=[point[0]],
            y=[point[1]],
            line=dict(
                color=color,
                width=10
            ),
            mode="markers"
        )
        self.data.append(trace)

    def draw(self,O,trees,path=None,auto_open=True):
        self.plot_tree(trees)
        self.plot_obstacles(O)
        self.plot_start_end(x_init, "orange")
        self.plot_start_end(x_goal, "green")
        if path is not None:
            print("Path:\n",path)
            self.plot_path(path)
        py.offline.plot(self.fig, filename=self.filename, auto_open=auto_open)

# --------------------- Plot --------------------- #
# --------------------- Double Tree RRT* --------------------- #

class DoubleTreeRRTStar:
    def __init__(self, X, Q, x_init, x_goal, max_samples, rewire_count=None):       
        self.X = X
        self.samples_taken = 0
        self.max_samples = max_samples
        self.Q = Q
        self.r = 1
        self.prc = 0.1
        self.x_init = x_init
        self.x_goal = x_goal
        self.rewire_count = rewire_count if rewire_count is not None else 0
        self.c_best = float('inf')  # Best Cost      
        self.sigma_best = None  # Best Solution
        self.swapped = False
        self.trees = []  # List of all trees
        self.add_tree()  # Add initial tree

    def connect_trees(self, a, b, x_new, L_near):
        for c_near, x_near in L_near:
            c_tent = c_near + path_cost(self.trees[a].E, self.x_init, x_new)
            if c_tent < self.c_best and self.X.collision_free(x_near, x_new, self.r):
                self.trees[b].V_count += 1
                self.trees[b].E[x_new] = x_near
                self.c_best = c_tent
                sigma_a = self.reconstruct_path(a, self.x_init, x_new)
                sigma_b = self.reconstruct_path(b, self.x_goal, x_new)
                del sigma_b[-1]
                sigma_b.reverse()
                self.sigma_best = sigma_a + sigma_b

                break

    def swap_trees(self):
        self.trees[0], self.trees[1] = self.trees[1], self.trees[0]
        self.x_init, self.x_goal = self.x_goal, self.x_init
        self.swapped = not self.swapped

    def unswap(self):
        if self.swapped:
            self.swap_trees()

        if self.sigma_best is not None and self.sigma_best[0] is not self.x_init:
            self.sigma_best.reverse()

    def get_nearby_vertices(self, tree, x_init, x_new):
        X_near = self.nearby(tree, x_new, self.current_rewire_count(tree))
        L_near = [(path_cost(self.trees[tree].E, x_init, x_near) + distance(x_near, x_new), x_near) for
                  x_near in X_near]
        L_near.sort(key=itemgetter(0))

        return L_near

    def rewire(self, tree, x_new, L_near):
        for _, x_near in L_near:
            curr_cost = path_cost(self.trees[tree].E, self.x_init, x_near)
            tent_cost = path_cost(self.trees[tree].E, self.x_init, x_new) + distance(x_new, x_near)
            if tent_cost < curr_cost and self.X.collision_free(x_near, x_new, self.r):
                self.trees[tree].E[x_near] = x_new

    def connect_shortest_valid(self, tree, x_new, L_near):
        for c_near, x_near in L_near:
            if c_near + distance(x_near, self.x_goal) < self.c_best and self.connect_to_point(tree, x_near, x_new):
                break

    def current_rewire_count(self, tree):
        if self.rewire_count is None:
            return self.trees[tree].V_count

        return min(self.trees[tree].V_count, self.rewire_count)

    def add_tree(self):
        self.trees.append(Tree(self.X))

    def add_vertex(self, tree, v):
        self.trees[tree].V.insert(0, v + v, v)
        self.trees[tree].V_count += 1  
        self.samples_taken += 1  

    def add_edge(self, tree, child, parent):
        self.trees[tree].E[child] = parent

    def nearby(self, tree, x, n):
        return self.trees[tree].V.nearest(x, num_results=n, objects="raw")

    def get_nearest(self, tree, x):
        return next(self.nearby(tree, x, 1))

    def new_and_near(self, tree, q):
        x_rand = self.X.sample_free()
        x_nearest = self.get_nearest(tree, x_rand)
        x_new = self.bound_point(steer(x_nearest, x_rand, q[0]))
        if not self.trees[tree].V.count(x_new) == 0 or not self.X.check_free(x_new):
            return None, None
        self.samples_taken += 1
        return x_new, x_nearest

    def connect_to_point(self, tree, x_a, x_b):
        if self.trees[tree].V.count(x_b) == 0 and self.X.collision_free(x_a, x_b, self.r):
            self.add_vertex(tree, x_b)
            self.add_edge(tree, x_b, x_a)
            return True
        return False

    def reconstruct_path(self, tree, x_init, x_goal):
        path = [x_goal]
        current = x_goal
        if x_init == x_goal:
            return path
        while not self.trees[tree].E[current] == x_init:
            path.append(self.trees[tree].E[current])
            current = self.trees[tree].E[current]
        path.append(x_init)
        path.reverse()
        return path

    def bound_point(self, point):
        point = np.maximum(point, self.X.dimension_lengths[:, 0])
        point = np.minimum(point, self.X.dimension_lengths[:, 1])
        return tuple(point)

    def rrt_star_bidirectional(self):
        # tree a
        self.add_vertex(0, self.x_init)
        self.add_edge(0, self.x_init, None)

        # tree b
        self.add_tree()
        self.add_vertex(1, self.x_goal)
        self.add_edge(1, self.x_goal, None)

        while True:
            for q in self.Q:  
                for i in range(q[1]):  
                    x_new, x_nearest = self.new_and_near(0, q)
                    if x_new is None:
                        continue

                    # Get cost-to-come and nearest vertex
                    L_near = self.get_nearby_vertices(0, self.x_init, x_new)

                    # Connect to shortest valid vertex
                    self.connect_shortest_valid(0, x_new, L_near)

                    if x_new in self.trees[0].E:
                        self.rewire(0, x_new, L_near)
                        L_near = self.get_nearby_vertices(1, self.x_goal, x_new)
                        self.connect_trees(0, 1, x_new, L_near)

                    if self.prc and random.random() < self.prc: 
                        print("Checking if can connect to goal at", str(self.samples_taken), "samples", end="\r")
                        if self.sigma_best is not None:
                            print("\nCan connect to goal")
                            self.unswap()
                            return self.sigma_best

                    if self.samples_taken >= self.max_samples:
                        self.unswap()
                        if self.sigma_best is not None:
                            print("\nCan connect to goal")
                            return self.sigma_best
                        else:
                            print("\nCould not connect to goal")
                        return self.sigma_best

            self.swap_trees()

# --------------------- Double Tree RRT* --------------------- #
# --------------------- Main --------------------- #

# Initializing the ros node
rospy.init_node('path_planning')
rospy.loginfo("Path planning node started")

# Selecting the map
print("Select the map:\n1. Map 1\n2. Map 2\n")
map_no=int(input("Enter the map number:"))

# Creating the map
Dimensions,Obstacles,map_image=create_map(map_no)
invert_y=map_image.shape[0]

# Selecting the start and goal points
cv.namedWindow("Map")
cv.setMouseCallback("Map", start_goal_selector)

print("\n\nSelect the start and goal points on the map\n")
while True:
    cv.imshow("Map",map_image)
    cv.waitKey(1)
    if start_end[0]==True and start_end[1]==True:
        cv.destroyAllWindows()
        break

start_time=time.time()

# Double Tree RRT* parameters
Q = np.array([(8, 8)])                          # Length of edges
max_samples = 2048                              # Max samples to take before timing out
rewire_count = 10                               # Number of nearby vertices to rewire

# Create Occupancy Grid(1-Obstacles, 0-Free Space)
X = OccupancyGrid(Dimensions, Obstacles)

# Create Double Tree RRT* object
rrt = DoubleTreeRRTStar(X, Q, x_init, x_goal, max_samples, rewire_count)
path = rrt.rrt_star_bidirectional()

# Plotting
plot = Plot("output")
plot.draw(Obstacles,rrt.trees,path,auto_open=True)

end_time=time.time()
print(f"Time taken to execute the code is: {(end_time-start_time)/60} minutes.")

#--------------------- ROS ---------------------#
rate = rospy.Rate(10)
move_robot(path)