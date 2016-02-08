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
      this.get('jsPlumbInstance').selectEndpoints().each(function (endpoint) {
        endpoint.removeClass('validDropPoint');
        endpoint.removeClass('invalidDropPoint');
      });
    });

    this.get('jsPlumbInstance').bind("connectionDrag", function (connection) {
      var sourceType = connection.endpoints[0].getParameter("type");
      var connections = this.get('jsPlumbInstance').selectEndpoints().each(function(endpoint) {
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
    this.get('jsPlumbInstance').draggable($(".project-container .window"), { grid: [20, 20], containment: "parent"});

    /*
     Connection click handler...
     */
    this.get('jsPlumbInstance').bind("click", function (conn, originalEvent) {
      if (originalEvent.ctrlKey) { // delete the connection on ctrl click
        jsPlumbInstance.detach(conn);
      }
    });

    this.set('targetEndpoint', {
        endpoint: "Dot",
        paintStyle: { fillStyle: "#7AB02C", radius: 11 },
        hoverPaintStyle: this.get('endpointHoverStyle'),
        dropOptions: { hoverClass: "hover", activeClass: "active" },
        isTarget: true
    });

    this.set('sourceEndpoint', {
      endpoint: "Dot",
        paintStyle: {
        strokeStyle: "#7AB02C",
          fillStyle: "transparent",
          radius: 7,
          lineWidth: 3
      },
      isSource: true,
        connector: [ "Flowchart", { stub: [40, 60], gap: 10, cornerRadius: 5, alwaysRespectStubs: true } ],
        connectorStyle: this.get('connectorPaintStyle'),
        hoverPaintStyle: this.get('endpointHoverStyle'),
        connectorHoverStyle: this.get('connectorHoverStyle'),
        dragOptions: {}
    });

  },

  jsPlumbifyModule(module) {
    // TODO: Refactor this?
    // make draggable
    Ember.$('#'+module.uWindowId)
      .offset({top: module.drawInfo.y, left: module.drawInfo.x})
      .width(module.drawInfo.width)
      .height(module.drawInfo.height);

    this.get('jsPlumbInstance').draggable(Ember.$("#"+module.uWindowId), { grid: [20, 20], containment: "parent" });

    console.log(module.uWindowId);

    // connections
    // TODO: Fix this!
    this.addInputsToWindow(module.uWindowId, module.inputs.toArray());
    //this.addOutputsToWindow(module.uWindowId, module.outputs.toArray());
  },


  addInputsToWindow(moduleId, inputs) {
    // add enpoints for every input
    for (var i = 0; i < inputs.length; i++) {
      var anchorId = moduleId + "input" + i.toString;
      var pos = [0, ((i+1)/(inputs.length +1)), -1, 0];

      var te = this.get('targetEndpoint');
      jsPlumbInstance.addEndpoint('#'+moduleId, te, {
        uuid: moduleId + "/" + inputs[i].name,
        anchor: pos,
        parameters: {
          type: inputs[i].message_type,
          name: inputs[i].name
        },
        maxConnections: 10,
        overlays: [
          [ "Label", {
            location: [-1.5, 0.5],
            label: inputs[i].name,
            cssClass: "endpoint-label"
          } ]
        ]
      });
    }
  },

  addOutputsToWindow(moduleId, outputs) {
    // add an endpoint for every output
    for (var i = 0; i < outputs.length; i++) {
      var anchorId = moduleId + "output" + i.toString;
      var pos = [1, ((i+1)/(outputs.length +1)), 1, 0];
      jsPlumbInstance.addEndpoint(moduleId, this.get('sourceEndpoint').toJSON(), {
        uuid: moduleId + "/" + outputs[i].name,
        anchor: pos,
        parameters: {
          type: outputs[i].message_type,
          name: outputs[i].name
        },
        maxConnections: 10,
        overlays: [
          [ "Label", {
            location: [-1.5, 0.5],
            label: outputs[i].name,
            cssClass: "endpoint-label"
          } ]
        ]
      });
    }
  }
});
