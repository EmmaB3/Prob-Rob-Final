# NAME: main.py
# AUTHOR: Emma Bethel
# PURPOSE: implementation for basic vision target recognition and approaching 
#          using a Misty II

from sheepdog import Sheepdog

MISTY_IP = '10.245.91.249'


def main():
    bot = Sheepdog(MISTY_IP)
    keep_going = True

    while keep_going:
        print('going')
        bot.halt()

        bot.update_target_pos()
        bot.drive_toward_target()

        print('keep going? (y/n)')
        keep_going = should_keep_going()

    bot.quit()


# PURPOSE: prompts for and parses user input to decide whether robot should
#          continue for another time step
# RETURNS: true if user wants robot to continue, false otherwise
def should_keep_going():
    i = input()
    if i == 'y':
        return True
    elif i == 'n':
        return False
    else:
        print('invalid input; please try again')
        return should_keep_going()


if __name__ == '__main__':
    main()
