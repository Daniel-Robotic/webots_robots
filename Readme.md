# A set of robots for the Webots simulator

### The following works are currently available in the set:

1. LBR KUKA iiwa 7 R800
2. LBR KUKA iiwa 14 R820 :gear:
3. LBR KUKA iiwa 7 med :gear:

### The following type of sensors is presented:

1. Intel Realsense D455 
2. Intel Realsense L515 :gear:

### EndEffectors:

1. Two-finger grip with a special frame for mounting the camera [IntelRealsense](https://www.intelrealsense.com/) :computer:
2. Calibration template

### Facilities:

1. Floor with different textures
2. Table for a collaborative robot
3. Workspace Limiter

### Configuration Webots
Git cloning:
```bash
https://github.com/Daniel-Robotic/webots_robots.git
```

VS code create folder `.vscode` and file `settings.json`
```json
{
	"python.analysis.extraPaths": [
        "<webots_path>"
    ],

    "python.autoComplete.extraPaths": [
		"<webots_path>"
	],
}
```


Install libs
```bash
python3 -m venv venv
pip3 install -r requirements.py
```

Activation venv core
`Tools -> Preferences -> Python Commands`: `/home/user/webots_robots/venv/bin/python`

---
All models are also located on the website [Webots Cloud](https://webots.cloud/proto)

[Contact us](mailto:grabardm@ml-dev.ru) to discuss models and webots.