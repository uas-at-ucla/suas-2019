import React, { Component } from 'react';
import './Home.css';
import Dashboard from '../Dashboard/Dashboard'
import MapScreen from '../MapScreen/MapScreen'

class Home extends Component {
  state = {
    isSidebarShown: true
  }

  componentDidMount() {
    this.refs.sidebar.addEventListener('transitionend', (event) => {
      if (event.target.id === 'sidebar') {
        this.refs.mapScreen.refs.map.refreshMapSize();
      }
    });
  }

  render() {
    return (
<div className="Home">
  <div id="sidebar_container"
       className={!this.state.isSidebarShown ? 'hidden' : null}>
    <div id="sidebar"
         ref="sidebar"
         className={!this.state.isSidebarShown ? 'hidden' : null}>
      <Dashboard close={this.hideSidebar.bind(this)}/>
    </div>
  </div>
  <MapScreen ref="mapScreen"
             isSidebarShown={this.state.isSidebarShown}
             showSidebar={this.showSidebar.bind(this)}/>
</div>
    );
  }

  showSidebar() {
    this.setState({isSidebarShown: true});
  }

  hideSidebar() {
    this.setState({isSidebarShown: false});
  }
}

export default Home;
