# A script that takes 3 axis rotation values from an imu and dsiplays them using pyfirmata
import argparse
import math
import sys
import time

import pyfirmata

# Parse arguments
parser = argparse.ArgumentParser(description='A script that takes 3 axis rotation values from an imu and dsiplays them using pyfirmata')
parser.add_argument('-p', '--port', type=str, default='/dev/ttyACM0', help='the serial port of the Arduino')
parser.add_argument('-f', '--frequency', type=float, default=100, help='the frequency of the loop')
parser.add_argument('-d', '--degrees', action='store_true', help='display the values in degrees')
args = parser.parse_args()

# Setup pyfirmata
board = pyfirmata.Arduino(args.port)

# Setup the iterator to avoid serial overflow
it = pyfirmata.util.Iterator(board)
it.start()

# Setup the pins
pins = [board.get_pin('a:0:i'), board.get_pin('a:1:i'), board.get_pin('a:2:i')]

# Setup the loop
dt = 1.0 / args.frequency

# Loop
while True:
    try:
        # Read the values
        values = [pin.read() for pin in pins]

        # Convert to degrees if needed
        if args.degrees:
            values = [math.degrees(v) if v is not None else None for v in values]

        # Print the values
        print('{:.3f} {:.3f} {:.3f}'.format(*values))

        # Sleep
        time.sleep(dt)

    except KeyboardInterrupt:
        board.exit()
        sys.exit()
 x = values
# print the z axis value obtained from th imu
z = print(x[2])
# a function that implements LQR algorithm
def lqr(A,B,Q,R):
    """Solve the continuous time lqr controller.
     
    dx/dt = A x + B u
     
    cost = integral x.T*Q*x + u.T*R*u
    """
    #ref Bertsekas, p.151
 
    #first, try to solve the ricatti equation
    X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
     
    #compute the LQR gain
    K = np.matrix(scipy.linalg.inv(R)*(B.T*X))
     
    eigVals, eigVecs = scipy.linalg.eig(A-B*K)
     
    return K, X, eigVals
# a function that implements the controller
def lqr_controller(A,B,Q,R):
    K, X, eigVals = lqr(A,B,Q,R)
    #print('K: ', K)
    #print('P: ', X)
    #print('eigVals: ', eigVals)
    return K
# a function that implements the observer
def lqr_observer(A,C,Q,R):
    K, X, eigVals = lqr(A.T,C.T,Q,R)
    #print('K: ', K)
    #print('P: ', X)
    #print('eigVals: ', eigVals)
    return K.T
# pass the variable z as the input matrix x of the LQR controller funtion
x = z
# pass the variable x as the input matrix x of the LQR observer function
y = x
# define the matrices A, B, C, Q, R for the LQR controller
A = np.matrix([[0, 1, 0], [0, 0, 0], [0, 0, 0]])
B = np.matrix([[0], [1], [0]])
C = np.matrix([[1, 0, 0]])
Q = np.matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
R = np.matrix([[1]])
# call the LQR controller function
K = lqr_controller(A,B,Q,R)
# call the LQR observer function
L = lqr_observer(A,C,Q,R)
# define the matrices A, B, C, D for the linear system
A = np.matrix([[0.0, 1.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
B = np.matrix([[0.0], [1.0], [0.0]])
C = np.matrix([[1.0, 0.0, 0.0]])
D = np.matrix([[0.0]])
