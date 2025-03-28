from web_backend import web
import multiprocessing
import time
from random import randint, uniform

if __name__ == '__main__':
    multiprocessing.freeze_support()
    
    manager = multiprocessing.Manager()
    button_flags = manager.dict()
    marks = manager.list() 


def main(button_flags, marks):
    while True:
        while not button_flags["start"]:
            pass
        for i in range(len(marks)):
            marks.pop()
        while button_flags["start"]:
            a = randint(0, 4)
            color = ''
            if a == 1:
                color = 'red'
            elif a == 2:
                color = 'green'
            elif a == 3:
                color = 'blue'
            else:
                color = 'yellow'
            a = randint(0, 3)
            if a == 1:
                form = 'square'
            elif a == 2:
                form = 'triangle'
            elif a == 3:
                form = 'circle'

            marks.append([round(randint(0, 3), 1), round(randint(0, 3), 1), color, form])
            time.sleep(1)


if __name__ == '__main__':    
    button_flags["start"] = False
    button_flags["stop"] = True
    button_flags["pause"] = False
    button_flags["killswitch"] = False

    main_thread = multiprocessing.Process(target=main, args=(button_flags, marks))
    main_thread.start()

    try:
        web.button_flags = button_flags
        web.marks = marks
        web.app.run(host='127.0.0.1', port=5000, debug=True)
    except Exception as e:
        print(f'error: {e}')
    finally:
        main_thread.terminate()
        main_thread.join()
