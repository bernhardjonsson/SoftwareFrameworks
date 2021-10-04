# SoftwareFrameworks
	1) Finds cubes and publishes positions - CubeFinder.node (Bernhard + David)
		a. If cube is within bounds of Bucket then don't publish its position
	2) Actual robot mover, subscribes to CubeFinder.node - CubeMover (JP + Jakob)
		a. Move to cube position
		b. Invoke grabber
		c. Move above bin
		d. Orient correctly (
		  pose_goal2.orientation.y =-0.707106781187
		  pose_goal2.orientation.w =0.707106781187
)
		e. Invoke Release
		f. Listen for Cube position

TEST!!
