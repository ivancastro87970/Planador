import pi_servo_hat
import time

# Ensaio: Mover os motores para o mínimo de extensão, esperar um tempo, mover para a máxima extensão e por último recolher.

# Initialize Constructor
testPitch = pi_servo_hat.PiServoHat() #Motor que controla o angulo de pitch
testRoll = pi_servo_hat.PiServoHat() #Motor que controla o Roll
testSeringa = pi_servo_hat.PiServoHat() #Motor que controla a água que entra/sai na seringa

# Restart Servo Hat (in case Hat is frozen/locked)
testPitch.restart()
testRoll.restart()
testSeringa.restart()

# Moves Servo Position
testPitch.move_servo_position(0, 0, 90) #Canal 0, move para angulo = 0, máxima extensão = 90
testRoll.move_servo_position(1, 0, 120) #Canal 1, move para angulo = 0, máxima extensão = 120
testSeringa.move_servo_position(2, 0, 180) #Canal 2, move para angulo = 0, máxima extensão = 180

# Pause 1 sec
time.sleep(1)

# Moves servo position
testPitch.move_servo_position(0, 90, 90) #Canal 0, move para angulo = 90, máxima extensão = 90
testRoll.move_servo_position(1, 120, 120) #Canal 1, move para angulo = 120, máxima extensão = 120
testSeringa.move_servo_position(2, 180, 180) #Canal 2, move para angulo = 180, máxima extensão = 180

# Pause 20 sec
time.sleep(20)

# Moves servo position
testPitch.move_servo_position(0, 0, 90) #Canal 0, move para angulo = 0, máxima extensão = 90
testRoll.move_servo_position(1, 0, 120) #Canal 1, move para angulo = 0, máxima extensão = 120
testSeringa.move_servo_position(2, 0, 180) #Canal 2, move para angulo = 0, máxima extensão = 180