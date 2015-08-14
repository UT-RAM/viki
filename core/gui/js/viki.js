var modules;
var jsPlumbInstance;  // to make instance globally available
var modulesInCanvas = [];

$(document).ready(function(){
    $('#connCheck').click(function(){
        send('"connection_check"');
        return false;
    });

    $('#updateModules').click(function(){
        updateStatus('Asking for modules');
        send('"ask_available_modules"');
        return false;
    });

    updateStatus('Asking for modules');
    send('"ask_available_modules"');
});

function updateStatus(msg) {
    var dt = new Date();
    var time =('0'  + dt.getHours()).slice(-2)+':'+('0' + dt.getMinutes()).slice(-2)+':'+('0' + dt.getSeconds()).slice(-2);
    $('#statusLabel').prepend(time + " - " + msg + "<br />");
}

function updateModules(modulelist) {
    updateStatus('Received modules');
    modules = modulelist;
    showModulesInPalette(modules);
    initPalette();
    updateStatus('Updated module panel')
}

function showModulesInPalette(modules) {
    $('#palette #list').html("");
    modules.forEach(function(module){
        $('#palette #list').append('<li class="module_palette" id="'+module.id+'"><img src="img/plugin.png" />'+module.id+'</li>');
    });
} 

function initPalette() {
    $(".module_palette").attr({
        "draggable" : "true"
    });

    $(".module_palette").on("dragstart", startDrag);

    $(".project-container").attr({
        "ondragover" : "allowDrop(event)",
        "ondrop" : "dropModule(event)"
    });
    
}

// this is the paint style for the connecting lines..
var connectorPaintStyle = {
        lineWidth: 4,
        strokeStyle: "#61B7CF",
        joinstyle: "round",
        outlineColor: "white",
        outlineWidth: 2
    },
// .. and this is the hover style.
    connectorHoverStyle = {
        lineWidth: 4,
        strokeStyle: "#216477",
        outlineWidth: 2,
        outlineColor: "white"
    },
    endpointHoverStyle = {
        fillStyle: "#216477",
        strokeStyle: "#216477"
    };

jsPlumb.ready(function () {

    jsPlumbInstance = jsPlumb.getInstance({
        // default drag options
        DragOptions: { cursor: 'pointer', zIndex: 2000 },
        // the overlays to decorate each connection with.  note that the label overlay uses a function to generate the label text; in this
        // case it returns the 'labelText' member that we set on each connection in the 'init' method below.
        ConnectionOverlays: [
            [ "Arrow", { location: 1 } ],
            [ "Label", {
                location: 0.1,
                id: "label",
                cssClass: "aLabel"
            }]
        ],
        Container: "project-container"
    });

    var basicType = {
        connector: "StateMachine",
        paintStyle: { strokeStyle: "red", lineWidth: 4 },
        hoverPaintStyle: { strokeStyle: "blue" },
        overlays: [
            "Arrow"
        ]
    };
    jsPlumbInstance.registerConnectionType("basic", basicType);

        init = function (connection) {
        };

    // suspend drawing and initialise.
    jsPlumbInstance.batch(function () {

        // listen for new connections; initialise them the same way we initialise the connections at startup.
        jsPlumbInstance.bind("connection", function (connInfo, originalEvent) {
            init(connInfo.connection);
        });

        // make all the window divs draggable
        jsPlumbInstance.draggable($(".project-container .window"), { grid: [20, 20] });

        //
        // listen for clicks on connections, and offer to delete connections on click.
        //
        jsPlumbInstance.bind("click", function (conn, originalEvent) {
            // if (confirm("Delete connection from " + conn.sourceId + " to " + conn.targetId + "?"))
            //   instance.detach(conn);
            //conn.toggleType("basic");
        });
    });

    jsPlumb.fire("jsPlumbDemoLoaded", jsPlumbInstance);

});


function addInputsToWindow(moduleId, inputs) {
    var targetEndpoint = {
        endpoint: "Dot",
        paintStyle: { fillStyle: "#7AB02C", radius: 11 },
        hoverPaintStyle: endpointHoverStyle,
        maxConnections: -1,
        dropOptions: { hoverClass: "hover", activeClass: "active" },
        isTarget: true
    };

    for (var i = 0; i < inputs.length; i++) {
        var anchorId = moduleId + "input" + i.toString;
        var pos = [0, ((i+1)/(inputs.length +1)), -1, 0];
        jsPlumbInstance.addEndpoint(moduleId, targetEndpoint, {
            anchor: pos,
            overlays: [
                [ "Label", {
                    location: [-1.5, 0.5],
                    label: inputs[i].name,
                    cssClass: "endpointTargetLabel"
                } ]
            ]
        });
    }
}

function addOutputsToWindow(moduleId, outputs) {

    var sourceEndpoint = {
        endpoint: "Dot",
        paintStyle: {
            strokeStyle: "#7AB02C",
            fillStyle: "transparent",
            radius: 7,
            lineWidth: 3
        },
        isSource: true,
        connector: [ "Flowchart", { stub: [40, 60], gap: 10, cornerRadius: 5, alwaysRespectStubs: true } ],
        connectorStyle: connectorPaintStyle,
        hoverPaintStyle: endpointHoverStyle,
        connectorHoverStyle: connectorHoverStyle,
        dragOptions: {}
    };

    for (var i = 0; i < outputs.length; i++) {
        var anchorId = moduleId + "output" + i.toString;
        var pos = [1, ((i+1)/(outputs.length +1)), 1, 0];
        jsPlumbInstance.addEndpoint(moduleId, sourceEndpoint, {
            anchor: pos,
            overlays: [
                [ "Label", {
                    location: [-1.5, 0.5],
                    label: outputs[i].name,
                    cssClass: "endpointSourceLabel"
                } ]
            ]
        });
    }
};

function updateModule(event) {
    var module = getModuleById(this.id);

    jsPlumbInstance.batch(function() {
        addInputsToWindow(module.id, module.inputs);
        addOutputsToWindow(module.id, module.outputs);
    });
    console.log(this.id);
}

function startDrag(ev) {
    ev = ev.originalEvent;
    updateStatus("dragging object: " + ev.target.id);
    ev.dataTransfer.setData("moduleId", ev.target.id);
}

function allowDrop(ev) {
    // updateStatus("allowing drop now..");
    ev.preventDefault();
}

function dropModule(ev) {
    updateStatus("dropped a module to the project-container");
    ev.preventDefault();

    // module id:
    // TODO: make unique
    var data = ev.dataTransfer.getData("moduleId");
    var modId = data;
    var uModId = guid();  // generate unique id
    
    $(".project-container").append('<div class="window" id="'+uModId+'"><strong>'+modId+'</strong><br/><br/></div>');
    
    // make draggable
    var instance = jsPlumbInstance;
    instance.draggable($(".project-container .window"), { grid: [20, 20] });

    // start module at correct position
    var width = $(".project-container .window").width();
    var height = $(".project-container .window").height();
    // TODO: adjust for when not gripping in the center
    var X = ev.pageX - 0.5*width;
    var Y = ev.pageY - 0.5*width;

    $(".project-container #"+uModId).offset({
        top : Y,
        left: X
    });

    // add to inCanvasArray
    var modToAdd = getModuleById(modId);
    modToAdd.uWindowId = uModId;
    modulesInCanvas.push(modToAdd);
   
    // connections
    jsPlumbInstance.batch(function() {
        addInputsToWindow(uModId, modToAdd.inputs);
        addOutputsToWindow(uModId, modToAdd.outputs);
    });

}

function getModuleById(Id) {
    for (var i=0; i <modules.length; i++) {
        if (modules[i].id == Id) {
            return modules[i];
        }
    }
}

function getModuleByUWindowId(uId) {
    for (var i=0; i<modulesInCanvas.length; i++) {
        if (modulesInCanvas[i].uWindowId == uId) {
            return modulesInCanvas[i];
        }
    }
}

function guid() {
    // generates a unique number
    function s4() {
        return Math.floor((1 + Math.random()) * 0x10000)
          .toString(16)
          .substring(1);
    }
    return s4() + s4() + '-' + s4() + '-' + s4() + '-' +
    s4() + '-' + s4() + s4() + s4();
}

function deleteWindowFromCanvas(uId) {
    // remove from jsPlumb
    jsPlumbInstance.remove(uId);
    // remove from array
    for (var i=0; i<modulesInCanvas.length; i++) {
        if (modulesInCanvas[i].uWindowId == uId) {
            modulesInCanvas.splice(i, 1);
        }
    }
    // update status
    updateStatus("window removed!");
}