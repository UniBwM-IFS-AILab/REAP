import msvcrt

def main():
    file_path = 'help.txt'

    with open(file_path, 'w') as file:
        msvcrt.locking(file.fileno(), msvcrt.LK_LOCK, 1)
        print('locking help file.')
        file.write('need help, lost in the mountains!\n')
        msvcrt.locking(file.fileno(), msvcrt.LK_UNLCK, 1)
        print('released the lock.')

if __name__ == "__main__":
    main()
