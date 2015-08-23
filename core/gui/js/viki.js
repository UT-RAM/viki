var modules;
var jsPlumbInstance;  // to make instance globally available
var modulesInCanvas = [];
var selectedModuleUid = null;
var generatedGUIDs = [];

$(document).ready(function(){
    // All links with an id starting with viki are buttons that expect a reaction from python. This process is automated: the python function with name equal to the id will run.
    $("a[id^=viki]").click(function(){
        statusmessage = $(this).data("statusmessage");
        if(typeof(statusmessage) != "undefined")
        {
            updateStatus(statusmessage);
        }
        send(JSON.stringify({name: $(this).attr('id'), value: false}));
        return false;
    });

    $("#makeNoRun").click(function(){
        send(JSON.stringify({name: "vikiMakeNoRun", value: getConfigXML(getConfig())}));
    });

    $("#makeAndRun").click(function(){
        send(JSON.stringify({name: "vikiMakeAndRun", value: getConfigXML(getConfig())}));
       });

    $('#module-filter-text').on('keydown change', filterModules);

    $("#italian").click(function(){
        // get size of contair
        var w = $('.project-container').width();
        var h = $('.project-container').height();
        
        // put image in page
        $('.project-container').prepend('<img id="marioImg" src="img/mario.png" />')

        // set start position
        $("#marioImg").css({
            position: "absolute",
            top: (0.5*h) + "px",
            left: 0 + "px"
            }).show();

        // move to right (calls delete function afterwards)
        $("#marioImg").animate({left: w + 'px'}, 3000, "linear", removeMario);
    });

    function removeMario() {
        // function to remove the mario added when italian language support is used
        $("#marioImg").remove();
    }

    // Manually request first module list.
    updateStatus('Asking for initial module list');
    send(JSON.stringify({name: "vikiRefreshModules", value: false}));
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
        var icon_path = 'img/plugin.png';
        if (module.meta.icon != null) {
            var icon_path = '../../' + module.path.substring(0, module.path.lastIndexOf("/")) + '/' + module.meta.icon;
        }
        $('#palette #list').append('<li class="module_palette '+module.type+'" id="'+module.id+'">'+
            '<img src="'+icon_path+'" /><h3>'+module.meta.name+'</h3>'+
            '<p class="description">'+module.meta.description+'</p>'+
            '<p class="type">type: '+module.type+'</p>'+
            '</li>');        
    });
} 

function filterModules(event) {
    var filter = $(this).val();
    if (filter == '') {
        $('.module_palette').slideDown(200);
    }
    $('.module_palette').filter(':not(:contains('+filter+'))').slideUp(200);
    $('.module_palette').filter(':contains('+filter+')').slideDown(200);
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
    
    $(document).on('click', '.window', onWindowClick);
    $(document).on('keydown', keyPressed);
    $(document).on('module_selected', onModuleSelect);
}

function enableStartCore() {
    $("#vikiStopRosCore").parent().hide();
    $("#vikiStartRosCore").parent().show();
}

function enableStopCore() {
    $("#vikiStopRosCore").parent().show();
    $("#vikiStartRosCore").parent().hide();
}

function onWindowClick(event) {
    var willSelect = ( selectedModuleUid !== $(this).attr('id') ); // only select if we click a non-selected module
    clearSelectedModule(); // make sure we only select a single module
    if (willSelect) {
        selectedModuleUid = $(this).attr('id');    
        $(this).attr('selected', 'selected');
        $(document).trigger("module_selected");
    }
}

function clearSelectedModule() {
    selectedModuleUid = null;
    $('.window').removeAttr('selected');
    // clear the properties pane
    $('#selectedWindowProperties tbody').empty();
    $('p#selectedWindowInfo').empty();
    // set text instead
    $('#selectedWindowProperties tbody').append('<tr><td colspan="2">No module selected.</td></tr>');
}

function deleteSelectedModule() {
    if (selectedModuleUid != null) {
        deleteWindowFromCanvas(selectedModuleUid);
        clearSelectedModule(); 
    }
}

function keyPressed(event) {
    switch (event.which) {
        case 46: // delete
        case 8: //backspace
            deleteSelectedModule();
            break;
    }
}

function onModuleSelect(event) {
    var selectedModule = getModuleByUWindowId(selectedModuleUid);
    $('p#selectedWindowInfo').html("<h3>"+selectedModule.id+"</h3><br><strong>Uid: </strong>"+selectedModule.uWindowId+"<br/>");
    var tbody = $('#selectedWindowProperties tbody');
    tbody.empty();

    for (var i=0; i < selectedModule.executables.length; i++) {
        var exe = selectedModule.executables[i];
        for (var j=0; j < exe.params.length; j++) {
            var param = exe.params[j];
            tbody.append('<tr><th>'+ param.name +'</th><td><input class="form-control" type="text" value="'+param.default+'"/></td></tr>');
        }
    }
    if (tbody.html() == '') {
        tbody.append('<tr><td colspan="2"><em>No params</em></td></tr>');
    }

    // save the params in the modulelist
    $('.form-control').blur(function (event) {
        var param = {};  // premake object
        param.name = $(this).parent().siblings().text(); // set name
        param.value = $(this).val();  // set value

        // check if the parameter has been set before
        var parameterWasSetBefore = false;
        for (var i=0; i<selectedModule.params.length; i++) {
            var setParam = selectedModule.params[i];
            if (setParam.name == param.name) {
                // update value if it has been set
                setParam.value = param.value;
                parameterWasSetBefore = true;
            }
        }
        if (parameterWasSetBefore == false) {
            // add if it hasnt been set
            selectedModule.params.push(param);
        }

        // for (var i=0; i<selectedModule.executables.length; i++){
        //     var exe = selectedModule.executables[i];
        //     for (var j=0; j<exe.params.length; j++) {
        //         var p = exe.params[j];
        //         if (p.name == paramName) {
        //             p.value = $(this).val();
        //         }
        //     }
        // }
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

    // suspend drawing and initialise.
    jsPlumbInstance.batch(function () {

        // listen for new connections; initialise them the same way we initialise the connections at startup.
        jsPlumbInstance.bind("beforeDrop", function (connInfo) {
            sourceType = connInfo.connection.endpoints[0].getParameter("type");
            targetType = connInfo.dropEndpoint.getParameter("type");

            if (sourceType != targetType) {
                updateStatus("Not able to connect endpoints of different types");
                return false;
            }

            return true;
        });

        jsPlumbInstance.bind("connectionDrag", function (connection) {
            var sourceType = connection.endpoints[0].getParameter("type");
            var connections = jsPlumbInstance.selectEndpoints({scope:'.project-container'}).each(function(endpoint) {
                console.log(endpoint.getParameter('type'));
            });
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
            parameters: {
                type: inputs[i].message_type,
                name: inputs[i].name
            },
            overlays: [
                [ "Label", {
                    location: [-1.5, 0.5],
                    label: inputs[i].name,
                    cssClass: "endpoint-label"
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
            parameters: {
                type: outputs[i].message_type,
                name: outputs[i].name
            },
            overlays: [
                [ "Label", {
                    location: [-1.5, 0.5],
                    label: outputs[i].name,
                    cssClass: "endpoint-label"
                } ]
            ]
        });
    }
};

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
    var data = ev.dataTransfer.getData("moduleId");
    var modId = data;
    var uModId = modId + guid();  // generate unique id
    
    $(".project-container").append('<div class="window" id="'+uModId+'"><span class="window_label">'+modId+'</span></div>');

    // add to inCanvasArray
    var modToAdd = getModuleById(modId);
    modToAdd.uWindowId = uModId;
    modToAdd.params = [];  // premake list for parameters
    modToAdd.args = [];  // placeholder for command line arguments
    modulesInCanvas.push(modToAdd);

    // start module at correct position
    var width = 100;
    var height = 20 + Math.max(modToAdd.inputs.length, modToAdd.outputs.length) * 20;
    var X = ev.pageX - 0.5*width;
    var Y = ev.pageY - 0.5*height;

    $(".project-container #"+uModId).offset({
        top : Y,
        left: X
    }).width(width)
        .height(height);

    // make draggable
    jsPlumbInstance.draggable($(".project-container .window"), { grid: [20, 20] });

    // connections
    addInputsToWindow(uModId, modToAdd.inputs);
    addOutputsToWindow(uModId, modToAdd.outputs);

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
    // generate until a not-used number has been found    
    do {
        var generatedId = "_" + ("" + (Math.random()*Math.pow(36,4) << 0).toString(36)).slice(-4);
    }
    while(generatedGUIDs.indexOf(generatedId) > -1)

    // save number in list
    generatedGUIDs.push(generatedId);

    return generatedId;
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

function getConfig() {
    var config = {};
    config.modsToAdd = [];
    config.connectsToAdd = [];
    $(".project-container").children().each(function() {
        uId = this.id;  // unique id for this element
        var mod = {};  // initialize module object
        
        // save all modules (only use role, type and id)
        var tempmod = getModuleByUWindowId(uId);  // get infor from unique id        
        mod.id = uId;  // save id
        mod.type = tempmod.id;  // save type
        mod.role = tempmod.type;  // save unique id
        mod.params = tempmod.params;  // add parameter list

        config.modsToAdd.push(mod);  // add to list of modules to add

        // all connections for this module
        var connects = jsPlumbInstance.getAllConnections(uId);
        for (var i=0; i<connects.length; i++) {
            var connectionToAdd = {};  // init connect to add object
            
            var connect = connects[i];  // this connect
            if (connect.targetId == uId) {  // only write connection if this module is target (avoids doubles)
                for (var j=0; j<connect.endpoints.length; j++) {  // loop across the (two) enpoints
                    endpoint = connect.endpoints[j];
                    // get source or target, prepend with module id and "/"
                    if (endpoint.isSource) {
                        connectionToAdd.pub = endpoint.elementId + "/" + endpoint.getParameter("name");
                    } else if (endpoint.isTarget) {
                        connectionToAdd.sub = endpoint.elementId + "/" + endpoint.getParameter("name");
                    } else {
                        console.log('Found an enpoint that is not a source, nor a target');
                        alert('error, please check console.log');
                    }                
                }    

                // save to connectionlist
                config.connectsToAdd.push(connectionToAdd);
            }
            
        }
    });
    return config;
}

function getConfigXML(config) {
    // create config XML 
    var configXML = document.createElement("configuration");
    configXML.setAttribute("id", "VIKI-imported-config");

    // add modules to the config XML
    for (var i=0; i<config.modsToAdd.length; i++) {
        var tempMod = config.modsToAdd[i];
        var modXML = document.createElement(tempMod.role);
        modXML.setAttribute("type", tempMod.type);
        modXML.setAttribute("id", tempMod.id);

        // loop through parameters
        for (var j=0; j<tempMod.params.length; j++) {
            // method 1: creates <param ... />
            // var paramXML = '<param name="'+tempMod.params[j].name+'" value="'+tempMod.params[j].value+'" //>';
            // modXML.innerHTML = paramXML;
            // console.log(modXML);


            // method 2: creates <param ...></param>
            var paramXML = document.createElement("param");
            paramXML.setAttribute("name", tempMod.params[j].name);
            paramXML.setAttribute("value", tempMod.params[j].value);
            // automatically no closing tag is created. We need to force create one, because python wants a nicely closed tag.
            // this comment does nothing other then force create a closing tag.
            paramXML.innerHTML = "<!-- comment -->";
            modXML.appendChild(paramXML);
            // console.log(paramXML);

        }
        // console.log(modXML);
        configXML.appendChild(modXML);
    }

    // add connects to the config XML
    for (var i=0; i<config.connectsToAdd.length; i++){
        var connectXML = document.createElement("connect");
        connectXML.setAttribute("publisher", config.connectsToAdd[i].pub);
        connectXML.setAttribute("listener", config.connectsToAdd[i].sub);
        configXML.appendChild(connectXML);
    }
    
    // return
    return configXML.outerHTML;
    // send(JSON.stringify({name: "vikiMake", value: configXML.outerHTML}));
}