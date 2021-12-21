from robotest.robotest.pi_nav.navigation import BallDetector

from .balldetect import BallDetector

detector = BallDetector("")

def main():
    # Run the ball detector
    img = []

    ball_detected = check_ball(img)

    if ball_detected:
        # Stopping navigation
        detector.msgdata = "st"

        while ball_detected:
            detector.msgdata = ball_detected

        # Resuming navigation
        detector.msgdata = "re"

def check_ball(img):
    """
    Algorithm to check whether ball is in frame,

    Returns
    command
        Commands sent to the motor
    """
    command = ''
    found = 1

    if found:
        return command

    # Not found return empty string
    return command

if __name__ == '__main__':
    while True:
        try:
            main()
        except Exception as e:
            print(e)
