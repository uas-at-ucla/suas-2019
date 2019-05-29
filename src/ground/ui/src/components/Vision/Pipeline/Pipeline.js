import React, { Component } from 'react';
import './Pipeline.css';

import { ExpansionPanel, ExpansionPanelSummary, ExpansionPanelDetails } from '@material-ui/core';
import ExpandMoreIcon from '@material-ui/icons/ExpandMore';

import Popover from '@material-ui/core/Popover';

import Grid from '@material-ui/core/Grid'
import { GridListTile, GridListTileBar } from '@material-ui/core';
import { List, ListItem } from '@material-ui/core'
// import Button from '@material-ui/core/Button';

// import data from somewhere else
var data = [{
  title : "Raw",
  images : [{
      imageID : "0ce662d93161",
      imageSrc : "https://s3.amazonaws.com/uifaces/faces/twitter/rem/128.jpg",
      recTime : "12:22:22",
      lat : 34.55,
      long : 34.112,
      alt : "3400",
      step : "raw"
  },
  {
      imageID : "0ce662d93161",
      imageSrc : "https://s3.amazonaws.com/uifaces/faces/twitter/glif/128.jpg",
      recTime : "12:22:22",
      lat : 34.55,
      long : 34.112,
      alt : "3400",
      shape : "star",
      letter : "A",
      orientation : "1",
      color : "blue",
      sentTime : "12:22:40"
  },
  {
      imageID : "0ce662d93161",
      imageSrc : "https://s3.amazonaws.com/uifaces/faces/twitter/glif/128.jpg",
  },
  {
      imageID : "0ce662d93161",
      imageSrc : "https://s3.amazonaws.com/uifaces/faces/twitter/glif/128.jpg",
  },
  {
      imageID : "0ce662d93161",
      imageSrc : "https://s3.amazonaws.com/uifaces/faces/twitter/glif/128.jpg",
  },
  {
      imageID : "0ce662d93161",
      imageSrc : "https://s3.amazonaws.com/uifaces/faces/twitter/glif/128.jpg",
  },
  {
      imageID : "0ce662d93161",
      imageSrc : "https://s3.amazonaws.com/uifaces/faces/twitter/rem/128.jpg",
  }]
},
{
  title : "Localized",
  images : [],
  input : [{
    imageID : "0ce662d93161",
    imageSrc : "https://s3.amazonaws.com/uifaces/faces/twitter/rem/128.jpg",
    recTime : "12:22:22",
    lat : 34.55,
    long : 34.112,
    alt : "3400",
    step : "localized"
  },
  {
    imageID : "0ce662d93161",
    imageSrc : "https://s3.amazonaws.com/uifaces/faces/twitter/glif/128.jpg",
    recTime : "12:22:22",
    lat : 34.55,
    long : 34.112,
    alt : "3400",
    shape : "star",
    letter : "A",
    orientation : "1",
    color : "blue",
    sentTime : "12:22:40"
  },
  {
    imageID : "0ce662d93161",
    imageSrc : "https://s3.amazonaws.com/uifaces/faces/twitter/rem/128.jpg",
    recTime : "12:22:22",
    lat : 34.55,
    long : 34.112,
    alt : "3400",
    step : "localized"
  }],
  output : [{
    imageID : "0ce662d93161",
    imageSrc : "https://s3.amazonaws.com/uifaces/faces/twitter/rem/128.jpg",
    recTime : "12:22:22",
    lat : 34.55,
    long : 34.112,
    alt : "3400",
    step : "localized"
  },
  {
    imageID : "0ce662d93161",
    imageSrc : "https://s3.amazonaws.com/uifaces/faces/twitter/glif/128.jpg",
    recTime : "12:22:22",
    lat : 34.55,
    long : 34.112,
    alt : "3400",
    shape : "star",
    letter : "A",
    orientation : "1",
    color : "blue",
    sentTime : "12:22:40"
  },
  {
    imageID : "0ce662d93161",
    imageSrc : "https://s3.amazonaws.com/uifaces/faces/twitter/rem/128.jpg",
    recTime : "12:22:22",
    lat : 34.55,
    long : 34.112,
    alt : "3400",
    step : "localized"
  }]
},
{
  title : "Shape Classifier",
  images : []
},
{
  title : "Letter Classifier",
  images : []
},
{
  title : "Color Classifier",
  images : []
},
{
  title : "Orientation Classifier",
  images : []
},
{
  title : "Interop Queue",
  images : []
}];

for (var i = 0; i < 50; i++) {
  const img = {
    imageID : "0ce662d93161",
    imageSrc : "./src/schedule.png",
  };
  data[0].images.push(img);
}

class Pagination extends React.Component {
  constructor() {
    super();
    this.state = {
      todos: ['a','b','c','d','e','f','g','h','i','j','k'],
      currentPage: 1,
      todosPerPage: 50
    };
    this.handleClick = this.handleClick.bind(this);
  }

  handleClick(event) {
    console.log(Number(event.target.id));
    if (Number(event.target.id) !== 0) {
      this.setState({
        currentPage: Number(event.target.id)
      });
    }
  }

  render() {
    const { currentPage, todosPerPage } = this.state;

    // Logic for displaying todos
    const indexOfLastTodo = currentPage * todosPerPage;
    const indexOfFirstTodo = indexOfLastTodo - todosPerPage;
    const currentImages = this.props.images.slice(indexOfFirstTodo, indexOfLastTodo);

    const renderImages = currentImages.map(img => (
      <Grid item>
        <ImageTile img={img} key={img.imageID} />
      </Grid>
    ));

    // Logic for displaying page numbers
    const pageNumbers = [];
    for (let i = 1; i <= Math.ceil(this.props.images.length / todosPerPage); i++) {
      pageNumbers.push(i);
    }

    const renderPageNumbers = pageNumbers.map(number => {
      return (
        <button
          key={number}
          onClick={this.handleClick}
          id={number}
        >
          {number}
        </button>
      );
    });

    return (
      <div>
        <Grid container>
          {renderImages}
        </Grid>
        <Grid container>
          {renderPageNumbers}
        </Grid>
      </div>
    );
  }
}

const ImageDetails = ({img}) => {
  return (
    <List dense={true}>
        <ListItem> ImageID: {img.imageID} </ListItem>
        <ListItem> Time Received: {img.recTime} </ListItem>
        <ListItem> Latitude: {img.lat} </ListItem>
        <ListItem> Longitude: {img.long} </ListItem>
        <ListItem> Altitude: {img.alt} </ListItem>
    </List>
  );
}

class ImageTile extends React.Component {
  state = {
    anchorEl: null,
  };
  handleClick = event => {
    this.setState({
      anchorEl: (this.state.anchorEl == null) ? event.currentTarget : null,
    });
  };

  render() {
    const { anchorEl } = this.state;
    const open = Boolean(anchorEl);
    return (
        <GridListTile 
          key={this.props.img.imageSrc}
          id={this.props.img.imageSrc}
          aria-owns={open ? 'simple-popper' : undefined}
          aria-haspopup="true"
          variant="contained"
          onClick={this.handleClick}
          component='ul'
        >
          <img src={this.props.img.imageSrc} alt={this.props.img.imageID}/>
          <GridListTileBar 
            title = {this.props.img.imageID}
          />

          <Popover
            id="simple-popper"
            open={open}
            anchorEl={anchorEl}
            onClose={this.handleClose}
            anchorOrigin={{
              vertical: 'bottom',
              horizontal: 'center',
            }}
            transformOrigin={{
              vertical: 'top',
              horizontal: 'center',
            }}
          >
            <ImageDetails img={this.props.img} />
          </Popover>
        </GridListTile>
    );
  }
}

const Pipeline = (props) => {
  return (
    <div className="Pipeline">
      {data.map(panel => (
          <ExpansionPanel>
            <ExpansionPanelSummary expandIcon={<ExpandMoreIcon />}>
              {panel.title} ({panel.images.length} images)
            </ExpansionPanelSummary>
            <ExpansionPanelDetails>
              <Pagination images={panel.images}/>
            </ExpansionPanelDetails>
          </ExpansionPanel>
      ))}
    </div>
  );
};

export default Pipeline;