import htmlPy
import os

class BackendConfig(htmlPy.Object):

  BASE_DIR = os.path.abspath(os.path.dirname(__file__))

  @htmlPy.Slot(result=str)
  def get_base_dir(self):
    return "file:///" + self.BASE_DIR + "/dist/"

app = htmlPy.AppGUI(title=u"VIKI GUI", maximized=True, developer_mode=True, plugins=True)

app.template_path = os.path.abspath("./dist/")
app.static_path = os.path.abspath("./dist/")

app.template = ("index.html", {})

app.bind(BackendConfig())

app.start()
