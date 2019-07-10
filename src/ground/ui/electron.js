// Modules to control application life and create native browser window
const { app, BrowserWindow, Menu, MenuItem, protocol } = require("electron");

const path = require("path");

// Keep a global reference of the window object, if you don't, the window will
// be closed automatically when the JavaScript object is garbage collected.
let mainWindow;

function createWindow() {
  // Fix asset loading as per https://stackoverflow.com/a/43423171
  protocol.interceptFileProtocol(
    "file",
    (request, callback) => {
      let url = request.url.substr(7); /* all urls start with 'file://' */
      if (request.url.startsWith("file:///static")) {
        callback(path.normalize(`${__dirname}/build/${url}`));
      } else {
        callback(url);
      }
    },
    err => {
      if (err) console.error("Failed to register file protocol");
    }
  );

  // Create the browser window.
  mainWindow = new BrowserWindow({
    width: 800,
    height: 600,
    icon: path.join(__dirname, "icon.png"),
    webPreferences: {
      nodeIntegration: true
    }
  });

  // and load the url of the app.
  const isDev = require("electron-is-dev");
  mainWindow.loadURL(
    isDev
      ? "http://localhost:3001"
      : `file://${path.join(__dirname, "build/index.html")}`
  );

  // Open the DevTools.
  // mainWindow.webContents.openDevTools()

  // Emitted when the window is closed.
  mainWindow.on("closed", () => {
    // Dereference the window object, usually you would store windows
    // in an array if your app supports multi windows, this is the time
    // when you should delete the corresponding element.
    mainWindow = null;
  });

  // Allow viewing devtools, copying, pasting, etc. in packaged app:
  if (!isDev) {
    const template = [
      new MenuItem({
        label: "Application",
        submenu: menuRoles.map(role => {
          return { role: role };
        })
      })
    ];
    const menu = Menu.buildFromTemplate(template);
    Menu.setApplicationMenu(menu);
  }
}

// This method will be called when Electron has finished
// initialization and is ready to create browser windows.
// Some APIs can only be used after this event occurs.
app.on("ready", createWindow);

// Quit when all windows are closed.
app.on("window-all-closed", function() {
  app.quit();
});

// In this file you can include the rest of your app's specific main process
// code. You can also put them in separate files and require them here.

// All available menu roles
const menuRoles = [
  "toggledevtools",
  "undo",
  "redo",
  "cut",
  "copy",
  "paste",
  "pasteandmatchstyle",
  "delete",
  "selectall",
  "reload",
  "forcereload",
  "resetzoom",
  "zoomin",
  "zoomout",
  "togglefullscreen",
  "window",
  "minimize",
  "close",
  "help",
  "about",
  "services",
  "hide",
  "hideothers",
  "unhide",
  "quit",
  "startspeaking",
  "stopspeaking",
  "close",
  "minimize",
  "zoom",
  "front"
];
