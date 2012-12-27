
define(["dojo/_base/declare",
        "dojo/_base/lang",
        "dijit/_WidgetBase",
        "dijit/form/Button",
        "dojo/on",
        "./NavigationControl",
        ],
function(declare,lang,widgetbase,Button,on,NavControl)
{
    var popup = declare("yujin_webtools.widgets.NavigationPopup",[widgetbase],
        {
            postCreate : function()
            {
                ros.on('connection',lang.hitch(this,this.onConnect));
                ros.on('close',lang.hitch(this,this.onClose));
                this.createButtons(); 
                
                dojo.connect(this.initButton,"onClick",this,"initPoseClicked");
                dojo.connect(this.goalButton,"onClick",this,"setGoalClicked");
                dojo.connect(this.cancelButton,"onClick",this,"cancelGoalClicked");

                this.initButton.setDisabled(true);
                this.goalButton.setDisabled(true);
                this.cancelButton.setDisabled(true);

                this.domNode.appendChild(this.initButton.domNode);
                this.domNode.appendChild(this.goalButton.domNode);
                this.domNode.appendChild(this.cancelButton.domNode);
            },

            onConnect : function() {
                this.initButton.setDisabled(false);
                this.goalButton.setDisabled(false);
                this.cancelButton.setDisabled(false);
            },

            onClose : function() {
                this.initButton.setDisabled(true);
                this.goalButton.setDisabled(true);
                this.cancelButton.setDisabled(true);
            },

            createButtons : function() 
            {
                this.initButton = new Button({ label:'Init Pose'});
                this.goalButton = new Button({label:'Set Goal'});
                this.cancelButton = new Button({label:'Cancel Goal'});
            },

            initPoseClicked : function(e){},
            setGoalClicked : function(e) {},
            cancelGoalClicked : function(e) {},

        });
        return popup;
});
