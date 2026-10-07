import configparser

def create_config():
    config = configparser.ConfigParser()

    config['Car_control'] = {
        'MAX_IA_SPEED': '4000',
        'BACKWARD_IA_SPEED': '-2000',
        'MAX_CONTROL_SPEED': '3',
        'MIN_CONTROL_SPEED': '-2',
        'MAX_ANGLE': '18',
        'MIN_ANGLE': '-18'
    }

    config['I2C'] = {
        'I2C_NUMBER_DATA_RECEIVED': '3',
        'I2C_SLEEP_RECEIVED': '0.1',
        'I2C_SLEEP_ERROR_LOOP': '1',
        'SLAVE_ADDRESS': '0x08'
    }

    config['Remote_control'] = {
        'PORT_REMOTE_CONTROL': '5556'
    }

    config['Camera'] = {
        'PORT_STREAMING_CAMERA': '8889',
        'STREAM_PATH': 'map',
        'SIZE_CAMERA_X': '1280',
        'SIZE_CAMERA_Y': '720',
        'FRAME_RATE': '30',
        'CAMERA_QUALITY': '10'
    }

    config['Car'] = {
        'CRASH_DIST': '110',
        'REAR_BACKUP_DIST': '100'
    }

    config['Lidar'] = {
        'LIDAR_DATA_AMPLITUDE': '1',
        'LIDAR_DATA_SIGMA': '45',
        'LIDAR_DATA_OFFSET': '0.5'
    }

    config['Screen_car'] = {
        'TEXT_HEIGHT': '11',
        'TEXT_LEFT_OFFSET': '3'
    }

    config['Paths'] = {
        'MODEL_PATH': '/home/intech/CoVAPSy/src/high_level/models/',
        'SITE_DIR_BACKEND': '/home/intech/CoVAPSy/src/high_level/src/site_controle'
    }

    config['Network'] = {
        'IP': '192.168.1.10',
        'LIDAR_IP': '192.168.0.10',
        'LIDAR_PORT': '10940'
    }

    config['Parameters'] = {
        'TEMPERATURE_STEER': '1',
        'TEMPERATURE_VITESSE': '1',
        'LOGGING_LEVEL': 'DEBUG'
    }

    config['Startup'] = {
        'CAMERA_STREAM_ON_START': 'True',
        'BACKEND_ON_START': 'True',
        'LIDAR_STREAM_ON_START': 'True',
        'LIMIT_CRASH_POINT': '10',
        'FREQUENCY_CRASH_DETECTION': '0.1',
        'FREQUENCY_REVERSE_DETECTION': '0.05',
        'LIMIT_REVERSE_COUNT': '6',
        'LIMIT_COUNT_WINDOW': '10'
    }

    with open('config.ini', 'w') as configfile:
        config.write(configfile)

if __name__ == '__main__':
    create_config()