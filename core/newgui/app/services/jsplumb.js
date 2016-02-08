import Ember from 'ember';

export default Ember.Service.extend({

  jsPlumbInstance: null,

  // define some styles for jsPlumb to work with
  connectorPaintStyle: {
      lineWidth: 4,
      strokeStyle: "#61B7CF",
      joinstyle: "round",
      outlineColor: "white",
      outlineWidth: 2
  },
  connectorHoverStyle: {
    lineWidth: 4,
    strokeStyle: "#216477",
    outlineWidth: 2,
    outlineColor: "white"
  },
  endpointHoverStyle: {
    fillStyle: "#216477",
    strokeStyle: "#216477"
  },

  init() {
    jsPlumbInstance = jsPlumb.getInstance({
      // default drag options
      DragOptions: { cursor: 'pointer', zIndex: 2000 },
      // the overlays to decorate each connection with.  note that the label overlay uses a function to generate the label text; in this
      // case it returns the 'labelText' member that we set on each connection in the 'init' method below.
      ConnectionOverlays: [
        [ "Arrow", { location: 1 } ]
      ],
      Container: "project-container"
    });
    this.set('jsPlumbInstance', jsPlumbInstance);

    // listen for new connections; initialise them the same way we initialise the connections at startup.
    this.get('jsPlumbInstance').bind("beforeDrop", function (connInfo) {
      //saveState();
      sourceType = connInfo.connection.endpoints[0].getParameter("type");
      targetType = connInfo.dropEndpoint.getParameter("type");

      if (sourceType !== targetType) {
        if (sourceType == "ANY" || targetType == "ANY") {
          return true;
        } else {
          console.log("Not able to connect endpoints of different types");
          return false;
        }
      }

      return true;
    });

    this.get('jsPlumbInstance').bind("beforeDetach", function(e){
      //saveState();
    });

    this.get('jsPlumbInstance').bind("connectionDragStop", function(connection) {
      jsPlumbInstance.selectEndpoints().each(function (endpoint) {
        endpoint.removeClass('validDropPoint');
        endpoint.removeClass('invalidDropPoint');
      });
    });

    this.get('jsPlumbInstance').bind("connectionDrag", function (connection) {
      var sourceType = connection.endpoints[0].getParameter("type");
      var connections = jsPlumbInstance.selectEndpoints().each(function(endpoint) {
        // Color code all target endpoints based on the source type
        if (endpoint.isTarget) {
          if (sourceType == "ANY") {
            endpoint.addClass('validDropPoint');
          } else if (endpoint.getParameter('type') == sourceType) {
            endpoint.addClass('validDropPoint');
          } else if (endpoint.getParameter('type') == "ANY") {
            endpoint.addClass('validDropPoint')
          } else {
            endpoint.addClass('invalidDropPoint');
          }
        }
      });
    });

    // make all the window divs draggable
    this.get('jsPlumbInstance').draggable($(".project-container .window"), { grid: [20, 20], start: saveState, containment: "parent"});

    /*
     Connection click handler...
     */
    this.get('jsPlumbInstance').bind("click", function (conn, originalEvent) {
      if (originalEvent.ctrlKey) { // delete the connection on ctrl click
        jsPlumbInstance.detach(conn);
      }
    });
  }
});
