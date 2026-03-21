from UI.App import App

def main():
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
if __name__ == "__main__":
    main()