import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches



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

    def generate_prm_nodes(self,n_nodes):
        nodes = []

        while len(nodes) < n_nodes:
            x = random.uniform(self.room_bounds["x_min"],self.room_bounds["x_max"])
            y = random.uniform(self.room_bounds["y_min"],self.room_bounds["y_max"])

            if ( self.is_point_collision_free(x,y)):
                nodes.append((x,y))

        return nodes
    
    def generate_plot(self,nodes):
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

        ax.scatter(x_coords,y_coords,c='blue',s=30,alpha=0.8,label="PRM NODES")

        ax.set_xlim(self.room_bounds['x_min']-20,self.room_bounds['x_max']+20)
        ax.set_ylim(self.room_bounds['y_min']-20,self.room_bounds['y_max']+20)
        ax.set_xlabel("X (cm)")
        ax.set_ylabel("Y (cm)")
        ax.set_title("PRM Node Generation")
        ax.legend()
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
    nodes = path_planner.generate_prm_nodes(50)
    path_planner.generate_plot(nodes)

if __name__ == "__main__":
    main()