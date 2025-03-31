from flask import Flask, jsonify, request

app = Flask(__name__)

data = []

send_data = []

@app.route('/data', methods=['POST'])
def receive_data():
    _data = request.json
    data.append(_data)
    return jsonify({"msg": "got data", "data": _data}), 200


@app.route('/data', methods=['GET'])
def get_data():
    return jsonify({"data": data}), 200


app.run(host='127.0.0.1', port=5000, debug=True)
