from flask import Flask, request, jsonify
import configparser 
app = Flask(__name__)

@app.route("/data", methods=['POST'])

def get_and_write():

	data = request.get_json()

	if data is None:
		return jsonify({"error": "Request body must be JSON"}), 400

	val_max_ai = data.get("val_max_ai")

	config = configparser.ConfigParser()
	config.read('config.ini')

	config['CarControl']['MAX_IA_SPEED'] = str(val_max_ai)

	with open('/home/intech/CoVAPSy/src/high_level/src/high_level/autoconfig.ini', 'w') as configfile:
		config.write(configfile)

	return jsonify({"status": "success","new_value": val_max_ai}), 200

if __name__ == '__main__':
	app.run(debug=True)