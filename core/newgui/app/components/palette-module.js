import Ember from 'ember';

export default Ember.Component.extend({
    module: {},
    canvasState: Ember.inject.service(),
    icon: "",

    dragStart(event) {
      event.dataTransfer.setData("id", this.get('module').id);
    },

    getIconHtml() {
        // Get a fancy image for the module, with a lot of fallbacks
      var icon_path;
      icon_path = 'img/plugin.png';
      var m = this.get('module');
        var image_html;
        if (m.meta.icon != null) {
          if (m.meta.icon.indexOf("glyphicon") == 0) {
            this.set('icon', m.meta.icon);
            image_html = "<i class='glyphicon "+m.meta.icon+"'></i>";
          } else {
            icon_path = '../../' + m.path.substring(0, m.path.lastIndexOf("/")) + '/' + m.meta.icon;
          }
        }

        return image_html;

        //TODO: Fix this for images as well!

        if (image_html == null) {
          image_html = '<img src="'+icon_path+'" />';
        }
    },

    hasGlyphicon: function() {
      console.log(this.get('icon'));
      return this.get('icon') !== "";
    }.property('icon')
});
