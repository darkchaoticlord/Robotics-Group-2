import brickpi3

BP = brickpi3.BrickPi3()


# Reset motor encoders to 0
def reset_encoders():
    BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))


# Get current encoding
def get_encoding():
    encoder_a = BP.get_motor_encoder(BP.PORT_A)
    encoder_b = BP.get_motor_encoder(BP.PORT_B)

    return (encoder_a, encoder_b)


def convert_cm_to_encoding(cm):
    return 0


# Drive in straight line
def drive_straight(cm):
    encoder_a, encoder_b = get_encoding()

    BP.set_motor_power(BP.PORT_A, -50)
    BP.set_motor_power(BP.PORT_B, -50)
    while encoder_a > -1000 and encoder_b > -1000:
        encoder_a, encoder_b = get_encoding()

    BP.set_motor_power(BP.PORT_A, 0)
    BP.set_motor_power(BP.PORT_B, 0)


def turn_right():
    encoder_a, encoder_b = get_encoding()

    BP.set_motor_power(BP.PORT_A, -50)
    BP.set_motor_power(BP.PORT_B, 50)
    while encoder_a > -700:
        encoder_a, encoder_b = get_encoding()

    BP.set_motor_power(BP.PORT_A, 0)
    BP.set_motor_power(BP.PORT_B, 0)


def main():
    reset_encoders()
    drive_straight(10)
    reset_encoders()


if __name__ == "__main__":
    main()
