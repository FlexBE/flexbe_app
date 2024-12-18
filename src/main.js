const { app, BrowserWindow } = require('electron')

function createWindow() {
  const win = new BrowserWindow({
      width: 1340,
      height: 830,
      minWidth: 1340,
      minHeight: 650,
      webPreferences: {
         nodeIntegration: true,
         contextIsolation: false 
      }
  });
  win.loadFile('src/window.html');
  win.setMenuBarVisibility(false) //idk the original didn't have one either
}

app.whenReady().then(() => {
  const win = createWindow();
});

app.on('open', () => {
    createWindow();
});

app.on('close', () => {
    process.exit();
});


