const webpack = require('webpack');
const config = {
  entry:  [
    __dirname + '/javascript/index.jsx',
  ],
  output: {
    path: __dirname + '/dist',
    filename: 'bundle.js',
    publicPath: '/client/'
  },
  resolve: {
    extensions: ['.js', '.jsx', '.css']
  },
  module: {
    rules: [{
      test: /\.jsx?$/,
      exclude: /node_modules/,
      loaders: ['babel-loader'],
    }, {
      test: /\.css$/,
      exclude: /node_modules/,
      loaders: ['style-loader', 'css-loader'],
    }]
  },
};

module.exports = config;
