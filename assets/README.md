# Assets Folder

This folder contains all the static assets for the EzBot Robotics Team website.

## Structure

- `images/` - Store all images here (logos, team photos, project images, etc.)
- `css/` - Additional CSS files (if needed)
- `js/` - Additional JavaScript files (if needed)
- `docs/` - Documentation files, PDFs, etc.

## Image Guidelines

When adding images to this folder:

1. **Optimize images** for web use (compress to reduce file size)
2. **Use descriptive filenames** (e.g., `team-photo-2025.jpg`, `robot-competition-win.png`)
3. **Recommended formats**:
   - JPEG for photos
   - PNG for graphics with transparency
   - SVG for logos and icons
4. **Recommended sizes**:
   - Hero images: 1920x1080px or similar
   - Team photos: 800x600px or similar
   - Project thumbnails: 400x300px or similar

## Usage in HTML

To use images from this folder in your HTML:

```html
<!-- For images in the images subfolder -->
<img src="assets/images/your-image.jpg" alt="Description">

<!-- For other assets -->
<link rel="stylesheet" href="assets/css/additional-styles.css">
<script src="assets/js/additional-scripts.js"></script>
```

## File Organization Tips

- Create subfolders for different types of content:
  - `images/team/` for team member photos
  - `images/projects/` for project images
  - `images/competitions/` for competition photos
  - `docs/` for downloadable documents
