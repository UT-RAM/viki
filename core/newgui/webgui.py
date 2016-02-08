import htmlPy
import os

app = htmlPy.AppGUI(title=u"VIKI GUI", maximized=True, developer_mode=True, plugins=True)

app.template_path = os.path.abspath("./dist/")
app.static_path = os.path.abspath("./dist/")

app.template = ("index.html", {})

app.start()
