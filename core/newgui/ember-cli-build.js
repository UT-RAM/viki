/*jshint node:true*/
/* global require, module */
var EmberApp = require('ember-cli/lib/broccoli/ember-app');

module.exports = function(defaults) {
  var app = new EmberApp(defaults, {
    // Add options here
  });

  // Use `app.import` to add additional libraries to the generated
  // output files.
  //
  // If you need to use different assets in different
  // environments, specify an object as the first parameter. That
  // object's keys should be the environment name and the values
  // should be the asset to use in that environment.
  //
  // If the library that you are including contains AMD or ES6
  // modules that you would like to import into your application
  // please specify an object with the list of modules as keys
  // along with the exports of each module as its value.

  // Own extra files
  app.import('vendor/css/viki.css');

  // Bower components extra stuff
  app.import('bower_components/jsPlumb/dist/js/jsPlumb-2.0.7-min.js');

  app.import('bower_components/bootstrap/dist/css/bootstrap.min.css');
  app.import('bower_components/bootstrap/dist/js/bootstrap.min.js');
  app.import('bower_components/bootstrap/dist/fonts/glyphicons-halflings-regular.ttf');
  app.import('bower_components/bootstrap/dist/fonts/glyphicons-halflings-regular.svg');
  app.import('bower_components/bootstrap/dist/fonts/glyphicons-halflings-regular.eot');
  app.import('bower_components/bootstrap/dist/fonts/glyphicons-halflings-regular.woff');
  app.import('bower_components/bootstrap/dist/fonts/glyphicons-halflings-regular.woff2');

  return app.toTree();
};
