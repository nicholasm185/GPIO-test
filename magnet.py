import time
import py_qmc5883l

sensor = py_qmc5883l.QMC5883L(output_data_rate=py_qmc5883l.ODR_100HZ, output_range=py_qmc5883l.RNG_8G)

def getMagnetStrength():
    return sensor.get_magnet_raw()[1]
