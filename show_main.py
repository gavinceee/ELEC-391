from UI.show_app import ShowApp


def main():
    app = ShowApp()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()


if __name__ == "__main__":
    main()