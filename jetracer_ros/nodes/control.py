import math

class LaneController:
    def __init__(self, config):
        self.speed = config.get('speed', 0.25)
        self.max_steering = config.get('max_steering', 0.8)
        self.kp = config.get('steering_kp', 1.2)
        self.kd = config.get('steering_kd', 0.05)
        self.lookahead_index = config.get('lookahead_index', 4)
        
        self.prev_error = 0.0

    def compute_command(self, path):
        """Compute steering command based on lateral error to a lookahead waypoint."""
        if not path.poses or len(path.poses) <= self.lookahead_index:
            return 0.0, 0.0

        # Extract target point
        target = path.poses[self.lookahead_index].pose.position
        
        # Lateral error is just the Y position of the waypoint in the robot frame
        # (Assuming robot is at origin pointing +X, Y is left)
        error = target.y
        
        # PD control
        derivative = error - self.prev_error
        steering = self.kp * error + self.kd * derivative
        
        self.prev_error = error
        
        # Clamp steering
        steering = max(min(steering, self.max_steering), -self.max_steering)
        
        return self.speed, steering
