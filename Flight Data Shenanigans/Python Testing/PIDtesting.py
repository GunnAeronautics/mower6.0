import time
import matplotlib.pyplot as plt



def compute():
	global integral
	global previous_error
	error = setpoint - current

		# Proportional term
	P = kp * error

		# Integral term
	integral += error
	I = ki * integral

		# Derivative term
	derivative = error - previous_error
	D = kd * derivative

		# PID output
	output = P + I + D

		# Save the current error for the next iteration
	previous_error = error

	return output

# Example usage
if __name__ == "__main__":
	kp = 0.05
	ki = 0.001
	kd = 0.05


	previous_error = 0
	integral = 0

	setpoint = 250.0  # Desired setpoint
	current = 0.0	# Current state
	currents = []
	outputs= []
	t = []
	for i in range(400):
		# Compute PID output
		output = compute()

		# Apply the output to the system (for simulation purposes)
		current += output

		# Print the current state
		print(f"Iteration {i}: Current = {current}")
		outputs.append(output)
		currents.append(current)
		t.append(i)
		# Simulate a delay (for demonstration purposes)
		#time.sleep(0.1)
	
	plt.plot(t, currents, label='Ax (°)')
	plt.plot(t, outputs, label='Ax (°)')

	plt.legend()
	plt.xlabel('Time')
	plt.ylabel('Data')
	plt.title('Flight Data')
	plt.show()