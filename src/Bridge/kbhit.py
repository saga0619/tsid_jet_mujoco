import os

if os.name == 'nt':  # Windows
    os = 'nt'
    import msvcrt
else:                # Posix (Linux, OS X)
    os = 'LINUX'
    import sys
    import termios
    import atexit
    from select import select

# special key definitions
ENTER = 10
ESC = 27
BACKSPACE = 127
TAB = 9


class KBHit:
    """ this class does the work """

    def __init__(self):
        """Creates a KBHit object to get keyboard input """

        if os == 'LINUX':
            # Save the terminal settings
            self.fd = sys.stdin.fileno()
            self.new_term = termios.tcgetattr(self.fd)
            self.old_term = termios.tcgetattr(self.fd)

            # New terminal setting unbuffered
            self.new_term[3] = (self.new_term[3] & ~
                                termios.ICANON & ~termios.ECHO)
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)

            # Support normal-terminal reset at exit
            atexit.register(self.set_normal_term)

    def set_normal_term(self):
        """ Resets to normal terminal.  On Windows does nothing """
        if os == 'LINUX':
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)

    def getch(self):
        """ Returns a keyboard character after kbhit() has been called """
        if os == 'nt':
            return msvcrt.getch().decode('utf-8')
        else:
            return sys.stdin.read(1)

    def kbhit(self):
        """ Returns True if keyboard character was hit, False otherwise. """
        if os == 'nt':
            return msvcrt.kbhit()
        else:
            dr, dw, de = select([sys.stdin], [], [], 0)
            return dr != []
