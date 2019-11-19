import React from 'react';
import CssBaseline from '@material-ui/core/CssBaseline';
import AppBar from '@material-ui/core/AppBar'
import ToolBar from '@material-ui/core/Toolbar'
import Button from '@material-ui/core/Button'
import './App.css';
import { Toolbar } from '@material-ui/core';

function App() {
  return (
    <div className="App">
      <CssBaseline />
      <header className="App-header">
        <AppBar>
          <ToolBar></ToolBar>
        </AppBar>
        <Button variant="contained" color="primary">
          Add Content...
        </Button>
      </header>
    </div>
  );
}

export default App;
