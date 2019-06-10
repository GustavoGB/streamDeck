from flask import Flask, request
from flask import Flask, request
from flask_restful import Resource, Api, reqparse
import time

import requests
import json
import threading

app = Flask(__name__)
api = Api(app)

parser = reqparse.RequestParser()
parser.add_argument('potenciometro')
parser.add_argument('botao')
parser.add_argument('id')
parser.add_argument('time')

def is_live_stream():
	while 1:
		url = "https://api.twitch.tv/kraken/streams/veguinho98?client_id=uf2ck23nxqeaohmgy79zamz0p65o1e"
		streamer_html = requests.get(url)
		streamer = json.loads(streamer_html.content)
		time.sleep(5)
		#MANDA STATUS DA LIVE
		if streamer["stream"] is not None:
			print("ESTA LIVE (L)")
		else:
			print("NAO ESTA LIVE (N)")
		status_check_stream = 0

todos = {}

class HelloWorld(Resource):
    def get(self):
        return {'hello': 'world'}
    def post(self):
        args = parser.parse_args()
        print(args)
        return {'hello':args['potenciometro'], 'hello':args['botao'], 'hello':['id'], 'hello':args['time']}

#class TodoSimple(Resource):
#    def get(self, todo_id):
#        return {todo_id: todos[todo_id]}
#
#    def put(self, todo_id):
#        todos[todo_id] = request.form['data']
#        return {todo_id: todos[todo_id]}

#api.add_resource(TodoSimple, '/<string:todo_id>')
api.add_resource(HelloWorld, '/')

if __name__ == '__main__':
    app.run(host='0.0.0.0',debug=True)
    while True:
        if(status_check_stream == 0):
            status_check_stream = 1
            threading.Thread(target=is_live_stream).start()

