import Ember from 'ember';

export function externalImage(params/*, hash*/) {
  let image_path = params[0];

  return BackendConfig.get_base_dir() + image_path;
}

export default Ember.Helper.helper(externalImage);
