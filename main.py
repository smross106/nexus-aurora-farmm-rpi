import numpy as np

class RobotPlaceholder(object):
    def __init__(self, frame_location, tool_location, rotation, tool_status=0, water_quantity=99999):

        # Location of bottom right corner of frame relative to ground. Same corner as water inlet. 3 dimensions, z=0. Metres
        self.frame_location = frame_location

        # Location of tool relative to bottom right corner of frame. 3 dimensions. Metres
        self.tool_location = tool_location

        # Rotation of frame seen as counter-clockwise from above. Same as complex plane. Assuming only rotation in one axis
        self.rotation = rotation

        # Status of the tool. Wil eventually become an Enum
        self.tool_status = 0

        # Litres of water (may change unit to seconds of pump time?) in storage tank
        self.water_quantity = 99999

        # Dimensions - these will ultimately be read out of a config file. [x, y, z]
        self.dimensions = [0.4, 1, 0.6]

        self.task_stack = []
        self.motion_stack = []
    
    def update_task_queue(self):
        """
        Changes status variables on tasks in the queue based on what's happened to them
        """
        task_ids_in_queue = []
        for i in self.motion_stack:
            task_ids_in_queue.append(i[5])
        
        for i in self.task_stack:
            count = sum(x==i["task_id"] for x in task_ids_in_queue)
            if count == i["motion_steps"]:
                i["task_status"] = "In queue"
            elif count > 0 and count < i["motion_steps"]:
                i["task_status"] = "In progress"
            elif count == 0:
                i["task_status"] = "Complete"

        new_task_stack = [s for s in self.task_stack if s["task_status"] != "Complete"]
        self.task_stack = new_task_stack

    def update_position(self):
        """
        When the state variables have been changed, update the position variables for rendering purposes

        TODO: replace all this nonsense with a single matrix multiplication operation
        """
        # Update the task queue
        self.update_task_queue()

        # Update the overall location nodes of the robot
        self.true_location = [0, 0, 0]

        self.true_location[2] = self.tool_location[2]

        self.true_location[0] = (self.frame_location[0]  
        + self.tool_location[0] * np.cos(np.deg2rad(self.rotation))
        - self.tool_location[1] * np.sin(np.deg2rad(self.rotation)))

        self.true_location[1] = (self.frame_location[1]  
        + self.tool_location[1] * np.cos(np.deg2rad(self.rotation))
        + self.tool_location[0] * np.sin(np.deg2rad(self.rotation)))


        # Location of bottom right, bottom left, top right, top left corners of the frame. All positions on ground level
        self.frame_corners = [list(self.frame_location),
        list(self.frame_location), 
        list(self.frame_location), 
        list(self.frame_location)]

        self.frame_corners[1][0] += (-self.dimensions[1] * np.sin(np.deg2rad(self.rotation)))
        self.frame_corners[1][1] += (self.dimensions[1] * np.cos(np.deg2rad(self.rotation)))

        self.frame_corners[2][0] += (self.dimensions[0] * np.cos(np.deg2rad(self.rotation)))
        self.frame_corners[2][1] += (self.dimensions[0] * np.sin(np.deg2rad(self.rotation)))

        self.frame_corners[3][0] += (self.dimensions[0] * np.cos(np.deg2rad(self.rotation))
        - self.dimensions[1] * np.sin(np.deg2rad(self.rotation)))
        self.frame_corners[3][1] += (self.dimensions[0] * np.sin(np.deg2rad(self.rotation))
        + self.dimensions[1] * np.cos(np.deg2rad(self.rotation)))

    def process_instruction_stack(self, tasks):
        """
        Takes a list of incoming tasks and adds them to the vehicle stack
        
        Currently placeholder (works on first-in-first-out), could one day do some clever prioritisation based on distance
        """
        for i in tasks:
            self.task_stack.append(i)

    def slice_motion(self, task):
        """
        Divide the motion required to reach a task destination into sections and feed them into the motion stack

        All motion commands are 5-axis
        [Tool x (forwards), tool y (across gantry), tool z(up and down), frame x (forwards), frame y]
        """

        steps = 0

        # Starts by copying either current location or location at end of motion stack
        # This means there are duplicates in the motion stack but it's a good way to safe the whole system
        if len(self.motion_stack) == 0:
            position = [self.tool_location[0], self.tool_location[1], self.tool_location[2],
            self.frame_location[0], self.frame_location[1]]
        else:
            position = self.motion_stack[-1]

        # First task is always to move tool to neutral position. Fully raised, right hand side.
        # This allows for endswitches to be triggered
        self.motion_stack.append(
            [position[0], 0, self.dimensions[2], 
            position[3], position[4], 
            task["task_id"]]
        )
        steps += 1

        # Determine required frame location for task
        # NOTE for now we're assuming the frame is constrained to x travel only. This may eventually not be the case
        # Aims to have the task location (or middle of task box, if it fits entirely within the travel)
        #   in the middle of the frame
        if task["location_type"] == "point":
            task_locations = [task["location"]]

        elif task["location_type"] == "grid":
            bounding_box = task["location"]
            grid_points = task["grid_points"]
            task_locations = []

            x_spacing = abs(bounding_box[1][0] - bounding_box[0][0]) / grid_points[0]
            y_spacing = abs(bounding_box[1][1] - bounding_box[0][1]) / grid_points[1]

            for x in range(grid_points[0]):
                for y in range(grid_points[1]):
                    task_locations.append(
                        [bounding_box[0][0] + x * x_spacing,
                        bounding_box[0][1] + y * y_spacing]
                    )
        
        centre_of_task_locations = [0, 0]
        valid_task_locations = []
        for i in task_locations:
            if i[1] < self.frame_location[1] or i[1] > (self.frame_location[1] + self.dimensions[1]):
                # Task location is inaccessible from the robot
                # Assuming it only moves in straight lines along the x axis
                pass
            else:
                valid_task_locations.append(i)
                centre_of_task_locations[0] += i[0]
                centre_of_task_locations[1] += i[1]
        
        centre_of_task_locations[0] /= len(valid_task_locations)
        centre_of_task_locations[1] /= len(valid_task_locations)
        
        centre_of_task_locations[0] -= self.dimensions[0]/2.

        # Move the frame so the centre lines up with the centre of the task envelope

        self.motion_stack.append([self.tool_location[0], 0, self.dimensions[2], 
            centre_of_task_locations[0], self.frame_location[1]
            , task["task_id"]])

        steps += 1
        
        for i in valid_task_locations:
            # For each task: move over it, lower down, fire tool, rise up

            self.motion_stack.append([
                i[0] - centre_of_task_locations[0], i[1], self.dimensions[2],
                centre_of_task_locations[0], position[4]
                , task["task_id"]])
            
            self.motion_stack.append([
                i[0] - centre_of_task_locations[0], i[1], 0,
                centre_of_task_locations[0], position[4]
                , task["task_id"]])
            
            self.motion_stack.append(["Tool","Tool","Tool","Tool","Tool", task["task_id"]])

            self.motion_stack.append([
                i[0] - centre_of_task_locations[0], i[1], self.dimensions[2],
                centre_of_task_locations[0], position[4]
                , task["task_id"]])
            
            steps += 4
    
        task["motion_steps"] = steps

task_queue = []

# PLaceholder tasks for now

task_queue.append({
    "operation_name":   "water",
    "task_status":      "Awaiting slicing",
    "location_type":    "point",
    "location":         [0.5,0.2],
    "toolhead":         "water_nozzle",
    "task_id":          0
})
task_queue.append({
    "operation_name":   "soil test",
    "task_status":      "Awaiting slicing",
    "location_type":    "grid",
    "location":         [[1.2,0.1],[1.5, 1.0]], # corners of square
    "grid_points":      [2, 3], # number of points in x and y
    "toolhead":         "soil_sampler",
    "task_id":          1
})
