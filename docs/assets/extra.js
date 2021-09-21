/* detect click or drag on zoomed images */
var dragged = false;
document.addEventListener("mousedown", () => (dragged = false));
document.addEventListener("mousemove", () => (dragged = true));
/* init view_bigimg */
function activateBigImg() {
    /* enable zoom-in */
    var figures = document.querySelectorAll("img");
    for (var i = 0; i < figures.length; i++) {
        figures[i].onclick = (e) => {
            if (e.target.nodeName === "IMG") {
                viewer.show(e.target.src);
            }
        };
    }
    /* click to close zoomed image */
    var containers = document.querySelectorAll("#iv-container .iv-image-view");
    for (var i = 0; i < containers.length; i++) {
        containers[i].onclick = () => {
            if (!dragged) {
                viewer.hide();
            }
        };
    }
}
const viewer = new ViewBigimg();
activateBigImg();

/* open external links in new tab */
function activateExternalLinks() {
    var links = document.links;
    for (var i = 0, linksLength = links.length; i < linksLength; i++) {
        if (links[i].hostname != window.location.hostname) {
            links[i].target = "_blank";
            links[i].setAttribute("rel", "noopener noreferrer");
            links[i].className += " externalLink";
        } else {
            links[i].className += " localLink";
        }
    }
}
activateExternalLinks();
