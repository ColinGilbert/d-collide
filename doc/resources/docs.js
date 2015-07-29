/* requires jquery >= 1.2.1
----------------------------------------------------------------------------*/

$(function () {
    $(".dynsection").hide();
    $(".dynheader").click(function () {
        // TODO: toggle following dynsection
        $(this).next(".dynsection").slideToggle();
    });
    
    // .tabs hack
    $(".tabs:first").addClass("tabs-first");
});