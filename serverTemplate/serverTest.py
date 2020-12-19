from flask import Flask, request
from flask_cors import CORS, cross_origin
from flask_restful import Resource, Api
from json import dumps
from flask_jsonpify import jsonify
import time

app = Flask(__name__)
CORS(app)

@app.route('/sendResult', methods=['POST'])
def test():
    if 'data' not in request.files:
        print('no files found')
    else:
        a = request.files['data']
        a.save('./serverTemplate/resultsDump/'+ str(time.time())+'.csv')
    return jsonify({'msg': 'success'})

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8080)