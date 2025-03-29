from flask import Flask, render_template, jsonify


class Web:
    def __init__(self):
        self.app = Flask(__name__)
        self.button_flags = None
        self.marks = None

    def create_routes(self):
        @self.app.route('/')
        def onstart():
            print('app started')
            return render_template('index.html')

        @self.app.route('/start_handle')
        def start_pressed():
            self.button_flags["start"] = True
            self.button_flags["stop"] = False
            return jsonify({'message': 'start handled'})

        @self.app.route('/stop_handle')
        def stop_pressed():
            self.button_flags["start"] = False
            self.button_flags["stop"] = True
            return jsonify({'message': 'stop handled'})

        @self.app.route('/pause_handle')
        def pause_pressed():
            self.button_flags["pause"] = not self.button_flags["pause"]
            return jsonify({'message': 'pause handled'})

        @self.app.route('/kill_handle')
        def kill_pressed():
            self.button_flags["killswitch"] = not self.button_flags["killswitch"]
            return jsonify({'message': 'kill handled'})

        @self.app.route('/get_marks')
        def send_marks():
            # print('received request, sent' + str(self.marks))
            return jsonify({'marks': list(self.marks)})


web = Web()
web.create_routes()
