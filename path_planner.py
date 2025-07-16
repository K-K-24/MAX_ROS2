import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import numpy as np
from queue import PriorityQueue
from matplotlib.ticker import MultipleLocator




class PathPlanner:
    def __init__(self,room_x_start,room_x_end,room_y_start,room_y_end,obstacles):
        self.obstacles = obstacles
        self.room_bounds = {
             "x_min":room_x_start,
             "x_max":room_x_end,
             "y_min":room_y_start,
             "y_max":room_y_end
        }

    def is_point_collision_free(self,x,y):
        for obstacle in self.obstacles:
            if(obstacle["type"]=="rectangle"):
                    if x >= obstacle["x_start"] and x <= obstacle["x_end"]:
                        if y >=obstacle["y_start"] and y <= obstacle["y_end"]:
                            return False
        return True

    def generate_prm_nodes(self,n_nodes,start,end):
        nodes = []

        nodes.append(start)

        while len(nodes) < n_nodes:
            x = int(random.uniform(self.room_bounds["x_min"],self.room_bounds["x_max"]))
            y = int(random.uniform(self.room_bounds["y_min"],self.room_bounds["y_max"]))

            if ( self.is_point_collision_free(x,y)):
                nodes.append((x,y))

        nodes.append(end)

        return nodes

    def build_roadmap(self,nodes,k_near):
      connections = []

      for i,node in enumerate(nodes):
        distances = []

        for j,other_node in enumerate(nodes):
            if i!=j:
              distance = ((node[0] - other_node[0])**2 + (node[1] - other_node[1])**2)**0.5
              distances.append((distance,j))

        distances.sort(key=lambda el:el[0])
        nearest = distances[:k_near]

        for dist,j in nearest:
          if self.is_collision_free(nodes[i],nodes[j]):
            connections.append((i,j))

      return connections

    def get_neighbours(self,current,connections,nodes):
      # return [ nodes[con[1]] for con in connections if nodes[con[0]] == current]
      neighbors = set()
      for con in connections:
          if nodes[con[0]] == current:
              neighbors.add(nodes[con[1]])
          elif nodes[con[1]] == current:  # CHECK BOTH DIRECTIONS!
              neighbors.add(nodes[con[0]])
      return list(neighbors)

    def euc_distance(self,node,other_node):
      distance = ((node[0] - other_node[0])**2 + (node[1] - other_node[1])**2)**0.5
      return distance


    def a_star_search(self,start,connections,goal,nodes):
      frontier = PriorityQueue()
      frontier.put((0,start))
      came_from = {}
      cost_so_far = {}
      came_from[start] = None
      cost_so_far[start] = 0

      while not frontier.empty():
        current_tuple = frontier.get()
        current = current_tuple[1]
        if current == goal:
          break

        for next in self.get_neighbours(current,connections,nodes):
          new_cost = cost_so_far[current] + self.euc_distance(current,next)

          if next not in cost_so_far or new_cost < cost_so_far[next]:
            cost_so_far[next] = new_cost
            priority = new_cost + self.euc_distance(goal,next)
            frontier.put((priority,next))
            came_from[next] = current

      if goal not in came_from:
        print("No path found - Start/end node is disconnected....")
        return []

      path = [goal]
      i = goal

      while i!=start:
        path.append(came_from[i])

        i = came_from[i]

      return path[::-1]


    def is_collision_free(self,pt1,pt2):
      for obstacle in self.obstacles:
        x_start = obstacle["x_start"]
        x_end = obstacle["x_end"]
        y_start = obstacle["y_start"]
        y_end = obstacle["y_end"]

        points = [(x_start,y_start),(x_start,y_end),(x_end,y_end),(x_end,y_start)]

        i = 0

        while(i < 4):
          j = (i+1)%4
          if (self.is_line_collision(pt1,pt2,points[i],points[j])):
            return False
          i += 1


      return True

    def within_range(self,t):
      return 0 <= t <= 1

    def is_line_collision(self,pt1,pt2,pt3,pt4):
      A1 = pt2[0] - pt1[0]
      A2 = pt2[1] - pt1[1]
      B1 = pt4[0] - pt3[0]
      B2 = pt4[1] - pt3[1]
      C1 = pt3[0] - pt1[0]
      C2 = pt3[1] - pt1[1]

      if(A1*B2 - A2*B1 == 0):
        return False

      t1 = (B2*C1 - B1*C2)/(A1*B2 - A2*B1)
      t2 = (A2*C1 - A1*C2)/(A1*B2 - A2*B1)

      if(self.within_range(t1) and self.within_range(t2)):
        return True

      return False

    def generate_plot(self,nodes,connections,path):
        fig,ax = plt.subplots(figsize=(12,8))

        room_rect = patches.Rectangle((
            self.room_bounds["x_min"],self.room_bounds["y_min"]),
            self.room_bounds["x_max"] - self.room_bounds["x_min"],
            self.room_bounds["y_max"] - self.room_bounds["y_min"],
            linewidth=2,edgecolor="black",facecolor="lightgray",alpha=0.3
        )

        ax.add_patch(room_rect)

        #obstacles
        for obstacle in self.obstacles:

            if(obstacle["type"]=="rectangle"):

                obstacle_rect = patches.Rectangle((
                obstacle["x_start"],obstacle["y_start"]),
                obstacle["x_end"] - obstacle["x_start"],
                obstacle["y_end"] - obstacle["y_start"],
                linewidth=2,edgecolor="red",facecolor="red",alpha=0.7
            )

                ax.add_patch(obstacle_rect)


        x_coords = [node[0] for node in nodes]
        y_coords = [node[1] for node in nodes]

# Plot start and end points with outlines
        ax.scatter(x_coords[0], y_coords[0], c='g', s=30,edgecolor='black', linewidth=2, label="Start")
        ax.scatter(x_coords[-1], y_coords[-1], c='r', s=30, edgecolor='black', linewidth=2, label="End")

        # Plot other PRM nodes
        ax.scatter(x_coords[1:-1], y_coords[1:-1], c='blue', s=30, alpha=0.8, label="PRM NODES")

        for connection in connections:
          i,j = connection
          x_points = [nodes[i][0],nodes[j][0]]
          y_points = [nodes[i][1],nodes[j][1]]

          ax.plot(np.array(x_points),np.array(y_points),'b-',linewidth=1)

        x_path_points = [pnt[0] for pnt in path]
        y_path_points = [pnt[1] for pnt in path]

        ax.plot(x_path_points, y_path_points, 'r-', linewidth=2, label="A* Path")

        # ax.plot(np.array(x_points),np.array(y_points),'b-',linewidth=1)



        ax.set_xlim(self.room_bounds['x_min']-30,self.room_bounds['x_max']+30)
        ax.set_ylim(self.room_bounds['y_min']-30,self.room_bounds['y_max']+30)
        ax.set_xlabel("X (cm)")
        ax.set_ylabel("Y (cm)")
        ax.set_title("PRM Node Generation")
        ax.legend()

        #Set major ticks to 60cm intervals
        ax.xaxis.set_major_locator(MultipleLocator(60))
        ax.yaxis.set_major_locator(MultipleLocator(60))
        ax.grid(True, alpha=0.3)
        # plt.show()
        plt.savefig("PRM NODES.png",dpi=300,bbox_inches="tight")





def main(args=None):
    room_x_start = 0
    room_x_end = 350

    room_y_start = 0
    room_y_end = 300


    obstacle1 = {
    "type": "rectangle",
    "name": "wall",
    "x_start": 310,
    "x_end": 350,
    "y_start": 0,
    "y_end": 40
}

    obstacle2 = {
        "type":"rectangle",
        "name":"table",
        "x_start":0,
        "x_end":70,
        "y_start":190,
        "y_end":300
    }

    obstacle3 = {
        "type":"rectangle",
        "name":"doormat",
        "x_start":250,
        "x_end":310,
        "y_start":255,
        "y_end":300
    }

    obstacles = [obstacle1,obstacle2,obstacle3]

    path_planner = PathPlanner(room_x_start,room_x_end,room_y_start,room_y_end,obstacles)
    start_node = (60,60)
    end_node =(340,280)
    nodes = path_planner.generate_prm_nodes(50,start_node,end_node)
    connections = path_planner.build_roadmap(nodes,5)

    print(nodes)
    print(connections)

    path = path_planner.a_star_search(start_node,connections,end_node,nodes)
    print(path)

    path_planner.generate_plot(nodes,connections,path)

    # print(connections)

if __name__ == "__main__":
    main()