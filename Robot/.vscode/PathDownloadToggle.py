from json import load, dump


settings_path = "C:/Users/jondr/AppData/roaming/Code/User/settings.json"

with open(settings_path, 'r') as f:
    settings = load(f)

if "ev3devBrowser.download.exclude" not in settings or settings["ev3devBrowser.download.exclude"] == "**/.*":
    settings["ev3devBrowser.download.exclude"] = "*Paths/*"
else:
    settings["ev3devBrowser.download.exclude"] = "**/.*"

with open(settings_path, 'w') as f:
    dump(settings, f)
