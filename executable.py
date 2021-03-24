import PythonUI.web_page_app as app
import Geometric_model.stewartPlatform as sp

if __name__ == "__main__":
    # Initialize platform
    stewart = sp()
    app.run(debug=True, use_reloader=False)